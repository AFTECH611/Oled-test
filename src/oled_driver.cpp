#include "oled_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace oled {

// ─────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────
namespace detail {

static float lerp(float a, float b, float t) noexcept { return a + t*(b-a); }

static float easeOut(float t) noexcept {
    t = std::max(0.f, std::min(1.f, t));
    return 1.f-(1.f-t)*(1.f-t)*(1.f-t);
}

static int64_t nowMs() noexcept {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static std::string sfmt(const char* fmt, ...) {
    char buf[128]; va_list ap;
    va_start(ap,fmt); std::vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
    return buf;
}

} // namespace detail

// ─────────────────────────────────────────────────────────────
//  5x7 ASCII font  (0x20 – 0x7E, 95 glyphs)
// ─────────────────────────────────────────────────────────────
static constexpr std::array<std::array<uint8_t,5>,95> kFont5x7{{
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},
}};

// ═════════════════════════════════════════════════════════════
//  FrameBuffer
// ═════════════════════════════════════════════════════════════
void FrameBuffer::setPixel(int x,int y,bool on){
    if(x<0||x>=kW||y<0||y>=kH) return;
    const int idx=(y>>3)*kW+x;
    const uint8_t bit=uint8_t(1u<<(y&7));
    if(on) buf_[idx]|=bit; else buf_[idx]&=~bit;
}
bool FrameBuffer::getPixel(int x,int y) const {
    if(x<0||x>=kW||y<0||y>=kH) return false;
    return (buf_[(y>>3)*kW+x]>>(y&7))&1u;
}
void FrameBuffer::drawHLine(int x,int y,int w){ for(int i=0;i<w;++i) setPixel(x+i,y); }
void FrameBuffer::drawVLine(int x,int y,int h){ for(int i=0;i<h;++i) setPixel(x,y+i); }
void FrameBuffer::drawLine(int x0,int y0,int x1,int y1){
    int dx=std::abs(x1-x0),dy=-std::abs(y1-y0);
    int sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx+dy;
    for(;;){
        setPixel(x0,y0);
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>=dy){err+=dy;x0+=sx;}
        if(e2<=dx){err+=dx;y0+=sy;}
    }
}
void FrameBuffer::drawRect(int x,int y,int w,int h){
    drawHLine(x,y,w); drawHLine(x,y+h-1,w);
    drawVLine(x,y,h); drawVLine(x+w-1,y,h);
}
void FrameBuffer::fillRect(int x,int y,int w,int h){
    for(int r=y;r<y+h;++r) drawHLine(x,r,w);
}
void FrameBuffer::drawCircle(int cx,int cy,int r){
    int xi=0,yi=r,d=3-2*r;
    auto p=[&](int a,int b){
        setPixel(cx+a,cy+b);setPixel(cx-a,cy+b);
        setPixel(cx+a,cy-b);setPixel(cx-a,cy-b);
        setPixel(cx+b,cy+a);setPixel(cx-b,cy+a);
        setPixel(cx+b,cy-a);setPixel(cx-b,cy-a);
    };
    while(yi>=xi){ p(xi,yi); if(d<0) d+=4*xi+6; else{d+=4*(xi-yi)+10;--yi;} ++xi; }
}
void FrameBuffer::fillCircle(int cx,int cy,int r){
    for(int yi=-r;yi<=r;++yi){
        int hw=int(std::sqrt(float(r*r-yi*yi)));
        drawHLine(cx-hw,cy+yi,2*hw+1);
    }
}
void FrameBuffer::invertRect(int x,int y,int w,int h){
    for(int r=y;r<y+h&&r<kH;++r)
        for(int c=x;c<x+w&&c<kW;++c)
            buf_[(r>>3)*kW+c]^=uint8_t(1u<<(r&7));
}

int FrameBuffer::drawChar(int x,int y,char c,int scale,bool inv){
    if(c<0x20||c>0x7E) c='?';
    const auto& g=kFont5x7[uint8_t(c)-0x20];
    for(int col=0;col<5;++col){
        uint8_t bits=g[col];
        for(int row=0;row<7;++row){
            bool on=(bits>>row)&1u;
            if(inv) on=!on;
            for(int sy=0;sy<scale;++sy)
                for(int sx=0;sx<scale;++sx)
                    setPixel(x+col*scale+sx,y+row*scale+sy,on);
        }
    }
    if(inv){
        for(int row=0;row<7*scale;++row)
            for(int sx=0;sx<scale;++sx)
                setPixel(x+5*scale+sx,y+row,true);
    }
    return 6*scale;
}
int FrameBuffer::drawString(int x,int y,std::string_view s,int scale,bool inv){
    int cx=x; for(char c:s) cx+=drawChar(cx,y,c,scale,inv); return cx-x;
}
void FrameBuffer::drawStringCenter(int y,std::string_view s,int scale,bool inv){
    drawString((kW-int(s.size())*charW(scale))/2,y,s,scale,inv);
}
void FrameBuffer::drawStringRight(int xr,int y,std::string_view s,int scale){
    drawString(xr-int(s.size())*charW(scale),y,s,scale);
}
void FrameBuffer::drawProgressBar(int x,int y,int w,int h,float pct){
    pct=std::max(0.f,std::min(1.f,pct));
    drawRect(x,y,w,h);
    if(int f=int((w-2)*pct);f>0) fillRect(x+1,y+1,f,h-2);
}
void FrameBuffer::drawScrollbar(int x,int y,int h,float pos,float vis){
    vis=std::max(0.1f,std::min(1.f,vis));
    int bh=int(h*vis);
    int by=y+int((h-bh)*pos);
    drawVLine(x,y,h); fillRect(x,by,2,bh);
}

// [FIX-F] Must use reference& - "auto x = rawMut()" copies the array!
void FrameBuffer::blitShifted(const FrameBuffer& src,int dx){
    auto& dst_buf=rawMut();           // reference to actual buffer
    const auto& src_buf=src.raw();
    for(int page=0;page<kPages;++page){
        uint8_t*       dst=dst_buf.data()+page*kW;
        const uint8_t* sr =src_buf.data()+page*kW;
        if(dx==0)              std::memcpy(dst,sr,kW);
        else if(dx>0&&dx<kW)  { std::memset(dst,0,dx);      std::memcpy(dst+dx,sr,kW-dx); }
        else if(dx<0&&dx>-kW) { int s=-dx; std::memcpy(dst,sr+s,kW-s); std::memset(dst+(kW-s),0,s); }
        else                   std::memset(dst,0,kW);
    }
}

// ═════════════════════════════════════════════════════════════
//  OledDisplay (SH1106 via I2C)
// ═════════════════════════════════════════════════════════════
OledDisplay::OledDisplay(std::string_view dev,uint8_t addr):addr_(addr),dev_(dev){}
OledDisplay::~OledDisplay(){ if(fd_>=0) ::close(fd_); }

bool OledDisplay::cmd(uint8_t c){
    const uint8_t buf[2]={0x00,c};
    return ::write(fd_,buf,2)==2;
}
bool OledDisplay::cmds(std::initializer_list<uint8_t> cs){
    for(uint8_t c:cs) if(!cmd(c)) return false; return true;
}

bool OledDisplay::init(){
    fd_=::open(dev_.c_str(),O_RDWR);
    if(fd_<0) return false;
    if(::ioctl(fd_,I2C_SLAVE,addr_)<0) return false;

    // SH1106 init (compatible with SSD1306 subset, but col offset differs)
    static constexpr uint8_t kSeq[]={
        0xAE,           // display off
        0xD5,0x80,      // clock divide
        0xA8,0x3F,      // multiplex 64
        0xD3,0x00,      // display offset 0
        0x40,           // start line 0
        0x8D,0x14,      // charge pump on
        0xA1,           // seg remap
        0xC8,           // COM flip
        0xDA,0x12,      // COM pins
        0x81,0xCF,      // contrast
        0xD9,0xF1,      // precharge
        0xDB,0x40,      // VCOM
        0xA4,           // follow RAM
        0xA6,           // normal (non-inverted)
        0xAF,           // display ON
    };
    for(uint8_t c:kSeq) if(!cmd(c)) return false;
    FrameBuffer blank; flush(blank);
    return true;
}

// [FIX-E] SH1106 has a 2-pixel hardware column offset.
//         col 0 in RAM = physical pixel 2. Must start at column address 2.
bool OledDisplay::writePage(int page,const uint8_t* data){
    cmd(uint8_t(0xB0|page)); // page address
    cmd(0x02);               // column low  nibble = 2  ← SH1106 offset
    cmd(0x10);               // column high nibble = 0

    uint8_t buf[129]; buf[0]=0x40;
    std::memcpy(buf+1,data,128);
    return ::write(fd_,buf,129)==129;
}

void OledDisplay::flush(const FrameBuffer& fb){
    const auto& raw=fb.raw();
    for(int p=0;p<kPages;++p) writePage(p,raw.data()+p*kW);
}
void OledDisplay::setContrast(uint8_t c){ cmds({0x81,c}); }
void OledDisplay::setOn(bool on){ cmd(on?0xAF:0xAE); }

// ═════════════════════════════════════════════════════════════
//  InputHandler
// ═════════════════════════════════════════════════════════════
struct InputHandler::GpioHandle {
    gpiod_chip* chip=nullptr;
    gpiod_line* line=nullptr;
    ~GpioHandle(){
        if(line) gpiod_line_release(line);
        if(chip) gpiod_chip_close(chip);
    }
    GpioHandle()=default;
    GpioHandle(const GpioHandle&)=delete;
    GpioHandle& operator=(const GpioHandle&)=delete;
};

static bool openGpio(const GpioPin& pin,InputHandler::GpioHandle& h,const char* consumer){
    h.chip=gpiod_chip_open(std::string{pin.chip}.c_str());
    if(!h.chip) return false;
    h.line=gpiod_chip_get_line(h.chip,pin.line);
    if(!h.line) return false;
    int rc=gpiod_line_request_input_flags(h.line,consumer,GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    if(rc<0) rc=gpiod_line_request_input(h.line,consumer);
    return rc>=0;
}

// [FIX-B] Corrected Gray-code quadrature table.
// Index = (prev_AB << 2) | curr_AB
// CW  sequence: 00 → 01 → 11 → 10 → 00
// CCW sequence: 00 → 10 → 11 → 01 → 00
//               prev\curr: 00   01   10   11
static constexpr int8_t kEncTable[16]={
     0,  -1,  +1,   0,   // prev=00
    +1,   0,   0,  -1,   // prev=01
    -1,   0,   0,  +1,   // prev=10
     0,  +1,  -1,   0,   // prev=11
};

InputHandler::InputHandler(Config cfg):cfg_(std::move(cfg)){}
InputHandler::~InputHandler(){ stop(); }

bool InputHandler::init(){
    static constexpr const char* kC="oled_ui";
    const GpioPin pins[]={cfg_.enc_a,cfg_.enc_b,cfg_.enc_btn,cfg_.confirm,cfg_.back};
    gpios_.clear();
    for(const auto& pin:pins){
        auto h=std::make_unique<GpioHandle>();
        if(!openGpio(pin,*h,kC)) return false;
        gpios_.push_back(std::move(h));
    }
    int a=gpiod_line_get_value(gpios_[0]->line);
    int b=gpiod_line_get_value(gpios_[1]->line);
    enc_prev_=uint8_t((a<<1)|b);
    return true;
}

void InputHandler::start(){
    if(running_.exchange(true)) return;
    thread_=std::thread(&InputHandler::loop,this);
}
void InputHandler::stop(){
    running_.store(false);
    if(thread_.joinable()) thread_.join();
}

int InputHandler::readLine(std::size_t idx) const {
    return gpiod_line_get_value(gpios_[idx]->line);
}

// [FIX-A] Input thread ONLY enqueues. No callback, no navigation, no render.
void InputHandler::enqueue(Ev ev){
    std::lock_guard<std::mutex> lk{q_mtx_};
    if(queue_.size()<32) queue_.push_back(ev);
}

// [FIX-A] Render thread calls this to get pending events.
std::vector<Ev> InputHandler::drainEvents(){
    std::lock_guard<std::mutex> lk{q_mtx_};
    std::vector<Ev> out(queue_.begin(),queue_.end());
    queue_.clear();
    return out;
}

void InputHandler::loop(){
    using namespace std::chrono;
    while(running_.load(std::memory_order_relaxed)){

        // ── Encoder ──────────────────────────────────────────
        const int a=readLine(0), b=readLine(1);
        const uint8_t curr=uint8_t((a<<1)|b);

        if(curr!=enc_prev_){
            const int8_t dir=kEncTable[(enc_prev_<<2)|curr];
            if(dir!=0){
                enc_accum_+=dir;
                // [FIX-C] threshold=2. Nếu encoder nhảy quá nhanh, đổi thành 4.
                if(enc_accum_>=2){ enqueue(Ev::EncCW);  enc_accum_=0; }
                else if(enc_accum_<=-2){ enqueue(Ev::EncCCW); enc_accum_=0; }
            }
            enc_prev_=curr;
        }

        // ── Buttons ──────────────────────────────────────────
        const int64_t now=detail::nowMs();

        auto handleBtn=[&](BtnState& bs,std::size_t gpio_idx,Ev short_ev,Ev long_ev){
            const bool raw=(readLine(gpio_idx)==0); // active-low with pull-up

            if(raw!=bs.last){ bs.last=raw; bs.change_ms=now; }

            // debounce window
            if((now-bs.change_ms)<cfg_.debounce_ms) return;

            if(raw&&!bs.stable){
                bs.stable=true; bs.press_ms=now; bs.long_fired=false;
            }
            else if(!raw&&bs.stable){
                bs.stable=false;
                if(!bs.long_fired&&short_ev!=Ev::None) enqueue(short_ev);
                bs.press_ms=0;
            }
            if(raw&&bs.stable&&!bs.long_fired&&long_ev!=Ev::None
               &&(now-bs.press_ms)>=cfg_.long_ms){
                enqueue(long_ev); bs.long_fired=true;
            }
        };

        handleBtn(bs_enc_,    2, Ev::EncPress,    Ev::None       );
        handleBtn(bs_confirm_, 3, Ev::Confirm,     Ev::ConfirmLong);
        handleBtn(bs_back_,   4, Ev::Back,         Ev::BackLong   );

        std::this_thread::sleep_for(milliseconds(cfg_.poll_ms));
    }
}

// ═════════════════════════════════════════════════════════════
//  UIScreen – title bar helper
// ═════════════════════════════════════════════════════════════
void UIScreen::drawTitleBar(FrameBuffer& fb,std::string_view label,int cur,int total){
    fb.fillRect(0,0,kW,9);
    fb.drawString(2,1,label,1,true);
    if(total>1){
        const auto s=detail::sfmt("%d/%d",cur,total);
        int tw=int(s.size())*FrameBuffer::charW();
        fb.drawStringRight(kW-1,1,s,1);
        fb.invertRect(kW-1-tw,0,tw,9);
    }
    fb.drawHLine(0,9,kW);
}

// ═════════════════════════════════════════════════════════════
//  MenuScreen
// ═════════════════════════════════════════════════════════════
const std::array<MenuScreen::Item,MenuScreen::kCount> MenuScreen::kItems={{
    {ScreenID::Joints,  "Joints",    "[J]"},
    {ScreenID::IMU,     "IMU",       "[I]"},
    {ScreenID::Joystick,"Joystick",  "[G]"},
    {ScreenID::LogInfo, "AimRT Log", "[L]"},
    {ScreenID::SBC,     "SBC Status","[S]"},
}};

MenuScreen::MenuScreen(NavCb nav):nav_(std::move(nav)){}
void MenuScreen::onEnter(){ pulse_=0.f; }

void MenuScreen::onInput(Ev ev){
    switch(ev){
    case Ev::EncCW:
        sel_=(sel_+1)%kCount;
        tsy_=float(sel_*12); pulse_=0.f; break;
    case Ev::EncCCW:
        sel_=(sel_-1+kCount)%kCount;
        tsy_=float(sel_*12); pulse_=0.f; break;
    case Ev::EncPress:
    case Ev::Confirm:
        if(nav_) nav_(kItems[sel_].sid,+1); break;
    default: break;
    }
}

void MenuScreen::update(float dt){
    sy_=detail::lerp(sy_,tsy_,std::min(1.f,dt*20.f));
    pulse_=std::fmod(pulse_+dt*2.f,2.f*3.14159265f);
}

void MenuScreen::render(FrameBuffer& fb) const {
    fb.clear();
    fb.fillRect(0,0,kW,9);
    fb.drawString(2,1,detail::sfmt("IP: %s",ip_.empty()?"---":ip_.c_str()),1,true);
    fb.drawHLine(0,9,kW);

    constexpr int kItemH=12,kY=11;
    for(int i=0;i<kCount;++i){
        int iy=kY+i*kItemH-int(sy_);
        if(iy<kY-kItemH||iy>kH) continue;
        bool sel=(i==sel_);
        if(sel) fb.fillRect(1,iy-1,kW-2,kItemH-1);
        fb.drawString(2, iy+2,kItems[i].icon, 1,sel);
        fb.drawString(16,iy+2,kItems[i].label,1,sel);
        if(sel) fb.drawString(kW-8,iy+2,">",1,true);
    }
    if(kCount>4){
        float pos=float(sel_)/float(kCount-1);
        fb.drawScrollbar(kW-2,kY,kH-kY,pos,4.f/float(kCount));
    }
}

// ═════════════════════════════════════════════════════════════
//  JointsScreen
// ═════════════════════════════════════════════════════════════
JointsScreen::JointsScreen(NavCb nav):nav_(std::move(nav)){}
void JointsScreen::onEnter(){ top_=0; soff_=tsoff_=0.f; }
void JointsScreen::setData(JointState d){ std::lock_guard<std::mutex> lk{mtx_}; data_=std::move(d); }

void JointsScreen::onInput(Ev ev){
    std::lock_guard<std::mutex> lk{mtx_};
    int n=int(data_.joints.size());
    switch(ev){
    case Ev::EncCW:
        if(top_<std::max(0,n-4)){ ++top_; tsoff_=float(top_*kRowH); } break;
    case Ev::EncCCW:
        if(top_>0){ --top_; tsoff_=float(top_*kRowH); } break;
    case Ev::Back:
        if(nav_) nav_(ScreenID::Menu,-1); break;
    default: break;
    }
}
void JointsScreen::update(float dt){
    soff_=detail::lerp(soff_,tsoff_,std::min(1.f,dt*18.f));
}
void JointsScreen::render(FrameBuffer& fb) const {
    fb.clear();
    std::lock_guard<std::mutex> lk{mtx_};
    drawTitleBar(fb,"JOINTS",1,5);
    int n=int(data_.joints.size());
    if(n==0){ fb.drawStringCenter(28,"No data"); return; }
    constexpr int kY0=11,kBarH=3;
    int y_off=int(soff_);
    for(int i=0;i<n;++i){
        int iy=kY0+i*kRowH-y_off;
        if(iy<kY0-kRowH||iy>kH) continue;
        const auto& j=data_.joints[i];
        fb.drawString(0, iy,j.name.substr(0,3));
        fb.drawString(20,iy,detail::sfmt("%+.2f",j.pos_rad));
        fb.drawString(68,iy,detail::sfmt("%.1f",j.temp_c));
        fb.drawChar(100,iy,'o'); fb.drawChar(106,iy,'C');
        float tpct=std::max(0.f,std::min(1.f,(j.temp_c-20.f)/60.f));
        fb.drawProgressBar(0,iy+kRowH-kBarH-1,kW-4,kBarH,tpct);
    }
    if(n>4){
        float pos=float(top_)/float(n-1);
        fb.drawScrollbar(kW-2,kY0,kH-kY0,pos,4.f/float(n));
    }
}

// ═════════════════════════════════════════════════════════════
//  IMUScreen
// ═════════════════════════════════════════════════════════════
IMUScreen::IMUScreen(NavCb nav):nav_(std::move(nav)){}
void IMUScreen::setData(IMUState d){ std::lock_guard<std::mutex> lk{mtx_}; data_=std::move(d); }
void IMUScreen::onInput(Ev ev){ if(ev==Ev::Back&&nav_) nav_(ScreenID::Menu,-1); }

void IMUScreen::update(float dt){
    float tx,ty;
    { std::lock_guard<std::mutex> lk{mtx_};
      tx=std::max(-1.f,std::min(1.f,data_.roll /30.f))*14.f;
      ty=std::max(-1.f,std::min(1.f,data_.pitch/30.f))*14.f; }
    float k=std::min(1.f,dt*10.f);
    sdot_x_=detail::lerp(sdot_x_,tx,k);
    sdot_y_=detail::lerp(sdot_y_,ty,k);
    static float tr_t=0.f;
    tr_t+=dt;
    if(tr_t>=0.05f){
        tr_t=0.f;
        trail_.push_back({int8_t(sdot_x_),int8_t(sdot_y_)});
        if(trail_.size()>12) trail_.pop_front();
    }
}
void IMUScreen::render(FrameBuffer& fb) const {
    fb.clear();
    drawTitleBar(fb,"IMU",2,5);
    IMUState d; { std::lock_guard<std::mutex> lk{mtx_}; d=data_; }
    fb.drawString(0,11,detail::sfmt("Gx%+5.1f",d.gx));
    fb.drawString(0,20,detail::sfmt("Gy%+5.1f",d.gy));
    fb.drawString(0,29,detail::sfmt("Gz%+5.1f",d.gz));
    fb.drawString(0,40,detail::sfmt("R %+5.1f",d.roll));
    fb.drawString(0,49,detail::sfmt("P %+5.1f",d.pitch));
    fb.drawString(0,58,detail::sfmt("Y %+5.1f",d.yaw));
    constexpr int cx=100,cy=38,r=20;
    fb.drawCircle(cx,cy,r);
    fb.drawHLine(cx-4,cy,9); fb.drawVLine(cx,cy-4,9);
    int tidx=0;
    for(auto [tx,ty]:trail_){
        int px=cx+tx,py=cy+ty;
        if(tidx>int(trail_.size())-4) fb.drawCircle(px,py,2);
        else fb.setPixel(px,py);
        ++tidx;
    }
    fb.fillCircle(cx+int(sdot_x_),cy+int(sdot_y_),3);
    fb.drawVLine(70,11,kH-11);
}

// ═════════════════════════════════════════════════════════════
//  JoystickScreen
// ═════════════════════════════════════════════════════════════
JoystickScreen::JoystickScreen(NavCb nav):nav_(std::move(nav)){}
void JoystickScreen::setData(JoystickState d){ std::lock_guard<std::mutex> lk{mtx_}; data_=std::move(d); }
void JoystickScreen::onInput(Ev ev){ if(ev==Ev::Back&&nav_) nav_(ScreenID::Menu,-1); }
void JoystickScreen::update(float dt){
    JoystickState d; { std::lock_guard<std::mutex> lk{mtx_}; d=data_; }
    float k=std::min(1.f,dt*15.f);
    slx_=detail::lerp(slx_,d.lx,k); sly_=detail::lerp(sly_,d.ly,k);
    srx_=detail::lerp(srx_,d.rx,k); sry_=detail::lerp(sry_,d.ry,k);
}
void JoystickScreen::render(FrameBuffer& fb) const {
    fb.clear();
    drawTitleBar(fb,"JOYSTICK",3,5);
    JoystickState d; { std::lock_guard<std::mutex> lk{mtx_}; d=data_; }
    constexpr int lx=23,ly=38,sr=17;
    fb.drawCircle(lx,ly,sr); fb.drawHLine(lx-3,ly,7); fb.drawVLine(lx,ly-3,7);
    fb.fillCircle(lx+int(slx_*(sr-3)),ly+int(sly_*(sr-3)),3);
    constexpr int rx=78,ry=38;
    fb.drawCircle(rx,ry,sr); fb.drawHLine(rx-3,ry,7); fb.drawVLine(rx,ry-3,7);
    fb.fillCircle(rx+int(srx_*(sr-3)),ry+int(sry_*(sr-3)),3);
    auto btn=[&](int bx,int by,char l,bool p){ fb.drawChar(bx,by,l); if(p) fb.invertRect(bx-1,by-1,7,9); };
    btn(110,13,'Y',d.btn_y); btn(119,20,'B',d.btn_b);
    btn(101,20,'X',d.btn_x); btn(110,27,'A',d.btn_a);
    fb.drawString(0, 11,d.btn_lb?"[LB]":" LB ");
    fb.drawString(50,11,d.btn_rb?"[RB]":" RB ");
    fb.drawString(43,56,d.btn_start ?"[ST]":" ST ");
    fb.drawString(80,56,d.btn_select?"[SE]":" SE ");
}

// ═════════════════════════════════════════════════════════════
//  LogScreen
// ═════════════════════════════════════════════════════════════
LogScreen::LogScreen(NavCb nav):nav_(std::move(nav)){}
void LogScreen::onEnter(){ auto_scroll_=true; }

void LogScreen::addLog(LogEntry e){
    std::lock_guard<std::mutex> lk{mtx_};
    logs_.push_back(std::move(e));
    if(int(logs_.size())>kMaxLogs) logs_.pop_front();
    if(auto_scroll_){
        int n=int(logs_.size());
        top_=std::max(0,n-4);
        tsoff_=soff_=float(top_*kRowH);
    }
}
void LogScreen::clear(){ std::lock_guard<std::mutex> lk{mtx_}; logs_.clear(); top_=0; soff_=tsoff_=0; }

void LogScreen::onInput(Ev ev){
    std::lock_guard<std::mutex> lk{mtx_};
    int n=int(logs_.size());
    switch(ev){
    case Ev::EncCW:
        if(top_<std::max(0,n-4)){ ++top_; tsoff_=float(top_*kRowH); auto_scroll_=(top_>=n-4); } break;
    case Ev::EncCCW:
        if(top_>0){ --top_; tsoff_=float(top_*kRowH); auto_scroll_=false; } break;
    case Ev::Back:
        if(nav_) nav_(ScreenID::Menu,-1); break;
    default: break;
    }
}
void LogScreen::update(float dt){
    soff_=detail::lerp(soff_,tsoff_,std::min(1.f,dt*18.f));
    blink_t_=std::fmod(blink_t_+dt*2.f,6.2831853f);
}
void LogScreen::render(FrameBuffer& fb) const {
    fb.clear();
    std::lock_guard<std::mutex> lk{mtx_};
    int n=int(logs_.size());
    drawTitleBar(fb,"AIMRT LOG",4,5);
    if(n>0&&auto_scroll_&&std::sin(blink_t_)>0.f) fb.fillCircle(kW-4,4,2);
    if(n==0){ fb.drawStringCenter(28,"No logs"); return; }
    constexpr int kY0=11;
    int y_off=int(soff_);
    for(int i=0;i<n;++i){
        int iy=kY0+i*kRowH-y_off;
        if(iy<kY0-kRowH||iy>kH) continue;
        const auto& e=logs_[i];
        char lc=(e.level==LogEntry::Level::Warn)?'W':(e.level==LogEntry::Level::Err)?'E':'I';
        bool hi=(e.level!=LogEntry::Level::Info);
        fb.drawChar(0,iy+1,lc,1,hi);
        auto mod=std::string_view{e.module}.substr(0,4);
        int msg_x=8+int(mod.size())*6+2;
        int max_ch=(kW-4-msg_x)/6;
        fb.drawString(8,    iy+1,mod);
        fb.drawString(msg_x,iy+1,std::string_view{e.msg}.substr(0,std::min(int(e.msg.size()),max_ch)));
        fb.drawHLine(0,iy+kRowH-1,kW-4);
    }
    if(n>4){
        float pos=float(top_)/float(std::max(1,n-4));
        fb.drawScrollbar(kW-2,kY0,kH-kY0,pos,4.f/float(n));
    }
}

// ═════════════════════════════════════════════════════════════
//  SBCScreen
// ═════════════════════════════════════════════════════════════
SBCScreen::SBCScreen(NavCb nav):nav_(std::move(nav)){}
void SBCScreen::setData(SBCStatus d){
    std::lock_guard<std::mutex> lk{mtx_};
    anim_core_.resize(d.core_pct.size(),0.f);
    data_=std::move(d);
}
void SBCScreen::onInput(Ev ev){ if(ev==Ev::Back&&nav_) nav_(ScreenID::Menu,-1); }
void SBCScreen::update(float dt){
    SBCStatus d; { std::lock_guard<std::mutex> lk{mtx_}; d=data_; }
    anim_t_+=dt;
    float k=std::min(1.f,dt*8.f);
    anim_core_.resize(d.core_pct.size(),0.f);
    for(std::size_t i=0;i<anim_core_.size();++i)
        anim_core_[i]=detail::lerp(anim_core_[i],d.core_pct[i]/100.f,k);
    float rp=(d.ram_total_mb>0.f)?d.ram_used_mb/d.ram_total_mb:0.f;
    anim_ram_=detail::lerp(anim_ram_,rp,k);
}
void SBCScreen::render(FrameBuffer& fb) const {
    fb.clear();
    SBCStatus d; { std::lock_guard<std::mutex> lk{mtx_}; d=data_; }
    drawTitleBar(fb,"SBC STATUS",5,5);
    auto tbuf=detail::sfmt("%.1foC",d.cpu_temp);
    int tw=int(tbuf.size())*FrameBuffer::charW();
    fb.drawStringRight(kW-1,1,tbuf,1);
    fb.invertRect(kW-1-tw,0,tw,9);
    constexpr int kBarW=54,kBarH=5,kLabelW=12;
    int ncores=int(std::min(anim_core_.size(),std::size_t{8}));
    for(int i=0;i<ncores;++i){
        int col=i>=4?1:0, row=i>=4?i-4:i;
        int bx=col*64+kLabelW, by=11+row*13, lx=col*64;
        fb.drawString(lx,by+1,detail::sfmt("C%d",i));
        fb.drawProgressBar(bx,by,kBarW,kBarH+2,anim_core_[i]);
        fb.drawString(bx+kBarW-24,by+kBarH+3,detail::sfmt("%3d%%",int(anim_core_[i]*100.f)));
    }
    int ram_y=11+4*13+2;
    fb.drawString(0,ram_y,detail::sfmt("RAM %.0f/%.0fMB",d.ram_used_mb,d.ram_total_mb));
    fb.drawProgressBar(0,ram_y+9,kW-1,5,anim_ram_);
}

// ═════════════════════════════════════════════════════════════
//  UIManager
// ═════════════════════════════════════════════════════════════
UIManager::UIManager(Config cfg):cfg_(std::move(cfg)){
    auto nav=[this](ScreenID id,int dir){ navigate(id,dir); };
    menu_     =std::make_unique<MenuScreen>    (nav);
    joints_   =std::make_unique<JointsScreen>  (nav);
    imu_      =std::make_unique<IMUScreen>     (nav);
    joystick_ =std::make_unique<JoystickScreen>(nav);
    log_      =std::make_unique<LogScreen>     (nav);
    sbc_      =std::make_unique<SBCScreen>     (nav);
    screens_[size_t(ScreenID::Menu)]     =menu_.get();
    screens_[size_t(ScreenID::Joints)]   =joints_.get();
    screens_[size_t(ScreenID::IMU)]      =imu_.get();
    screens_[size_t(ScreenID::Joystick)] =joystick_.get();
    screens_[size_t(ScreenID::LogInfo)]  =log_.get();
    screens_[size_t(ScreenID::SBC)]      =sbc_.get();
    cur_=menu_.get();
}
UIManager::~UIManager(){ stop(); }

bool UIManager::init(){
    disp_ =std::make_unique<OledDisplay>(cfg_.i2c_dev,cfg_.oled_addr);
    input_=std::make_unique<InputHandler>(cfg_.input);
    if(!disp_->init())  return false;
    if(!input_->init()) return false;
    // [FIX-A] NO callback set – render thread polls drainEvents() instead
    input_->start();
    cur_->onEnter();
    return true;
}

void UIManager::navigate(ScreenID id,int dir){
    UIScreen* next=screens_[size_t(id)];
    if(!next||next==cur_) return;
    cur_->render(fb_prev_);   // snapshot outgoing screen for transition
    cur_->onExit();
    prev_=cur_; cur_=next;
    in_trans_=true; trans_t_=0.f; trans_dir_=dir;
    cur_->onEnter();
}

void UIManager::tick(float dt){
    // [FIX-A] Drain input queue HERE in render thread, then dispatch to screen.
    //         This eliminates ALL cross-thread access to cur_ and screen state.
    for(Ev ev : input_->drainEvents()){
        if(cur_) cur_->onInput(ev);
    }

    if(cur_) cur_->update(dt);

    if(in_trans_){
        trans_t_+=dt;
        float t=std::min(trans_t_/trans_dur_,1.f);
        float et=detail::easeOut(t);
        if(cur_) cur_->render(fb_cur_);

        int new_dx,old_dx;
        if(trans_dir_>0){ new_dx=int((1.f-et)*kW);  old_dx=-int(et*kW); }
        else             { new_dx=-int((1.f-et)*kW); old_dx= int(et*kW); }

        fb_final_.blitShifted(fb_prev_,old_dx);
        fb_tmp_.blitShifted(fb_cur_,new_dx);

        // [FIX-G] Use raw pointers – "auto x = rawMut()" copies the array!
        uint8_t*       dst=fb_final_.rawMut().data();
        const uint8_t* src=fb_tmp_.raw().data();
        for(int i=0;i<kBufLen;++i) dst[i]|=src[i];

        disp_->flush(fb_final_);
        if(t>=1.f) in_trans_=false;
    } else {
        if(cur_){ cur_->render(fb_cur_); disp_->flush(fb_cur_); }
    }
}

void UIManager::run(){
    using namespace std::chrono;
    running_.store(true);
    const auto frame_dur=microseconds(1'000'000/cfg_.fps);
    auto last=steady_clock::now();
    while(running_.load(std::memory_order_relaxed)){
        auto now=steady_clock::now();
        float dt=std::max(0.001f,std::min(0.1f,
            duration_cast<microseconds>(now-last).count()/1'000'000.f));
        last=now;
        tick(dt);
        if(auto el=steady_clock::now()-now; el<frame_dur)
            std::this_thread::sleep_for(frame_dur-el);
    }
}

void UIManager::stop(){
    running_.store(false);
    if(input_) input_->stop();
}

void UIManager::setRobotIP      (std::string ip)   { menu_->setIP(std::move(ip)); }
void UIManager::setJointState   (JointState d)      { joints_->setData(std::move(d)); }
void UIManager::setIMUState     (IMUState d)        { imu_->setData(std::move(d)); }
void UIManager::setJoystickState(JoystickState d)   { joystick_->setData(std::move(d)); }
void UIManager::addLog          (LogEntry e)        { log_->addLog(std::move(e)); }
void UIManager::setSBCStatus    (SBCStatus d)       { sbc_->setData(std::move(d)); }

} // namespace oled