// sound.h  –  Section-based ambient sound system (Windows WinMM, no extra DLLs)
#pragma once
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

enum SectionID {
    SEC_NONE     = 0,
    SEC_ASSEMBLY = 1,
    SEC_PAINT    = 2,
    SEC_BODYSHOP = 3,
    SEC_SHOWROOM = 4,
    SEC_COUNT    = 5
};

class SoundSystem {
public:
    SectionID current = SEC_NONE;

    struct WavBuf { std::vector<uint8_t> d; };
    WavBuf buf[SEC_COUNT];   // indices 1..4 are used

    void init() {
        srand(42);
        makeAssembly (buf[SEC_ASSEMBLY]);
        makePaint    (buf[SEC_PAINT]);
        makeBodyShop (buf[SEC_BODYSHOP]);
        makeShowroom (buf[SEC_SHOWROOM]);
    }

    void setSection(SectionID s) {
        if (s == current) return;
        current = s;
        if (s == SEC_NONE) { PlaySoundA(NULL, NULL, 0); return; }
        auto& b = buf[s];
        if (!b.d.empty())
            PlaySoundA((LPCSTR)b.d.data(), NULL,
                       SND_MEMORY | SND_ASYNC | SND_LOOP);
    }

    void stop() { PlaySoundA(NULL, NULL, 0); current = SEC_NONE; }

private:
    static const int SR = 22050;

    // ── WAV helpers ──────────────────────────────────────────
    void hdr(WavBuf& wb, int n) {
        int ds = n * 2;
        auto& d = wb.d; d.clear(); d.reserve(44 + ds);
        w4(d,'R','I','F','F'); w32(d, 36 + ds);
        w4(d,'W','A','V','E');
        w4(d,'f','m','t',' '); w32(d,16);
        w16(d,1); w16(d,1);
        w32(d,SR); w32(d,SR*2);
        w16(d,2); w16(d,16);
        w4(d,'d','a','t','a'); w32(d,ds);
    }
    void w4(std::vector<uint8_t>& d,uint8_t a,uint8_t b,uint8_t c,uint8_t e)
        { d.push_back(a);d.push_back(b);d.push_back(c);d.push_back(e); }
    void w16(std::vector<uint8_t>& d,int16_t v)
        { d.push_back(v&0xFF);d.push_back((v>>8)&0xFF); }
    void w32(std::vector<uint8_t>& d,int32_t v)
        { d.push_back(v&0xFF);d.push_back((v>>8)&0xFF);
          d.push_back((v>>16)&0xFF);d.push_back((v>>24)&0xFF); }
    void s16(WavBuf& wb,int16_t v){ w16(wb.d,v); }
    float rnd(){ return (float)rand()/RAND_MAX*2.f-1.f; }

    // ── Assembly: rhythmic metallic clanking ─────────────────
    void makeAssembly(WavBuf& wb) {
        int n = SR*2; hdr(wb,n);
        for(int i=0;i<n;i++){
            float t=(float)i/SR;
            float ph=fmodf(t,0.35f);
            int16_t v=0;
            if(ph<0.04f){
                float dec=1.f-ph/0.04f;
                v=(int16_t)((rnd()*.7f+sinf(6.28f*520.f*ph)*.3f)*dec*26000);
            }
            float ph2=fmodf(t+0.17f,0.7f);
            if(ph2<0.025f){
                float dec=1.f-ph2/0.025f;
                v+=(int16_t)(rnd()*dec*14000);
            }
            v+=(int16_t)(sinf(6.28f*55.f*t)*1200);
            s16(wb,v);
        }
    }

    // ── Paint: spray hiss with rhythmic modulation ────────────
    void makePaint(WavBuf& wb) {
        int n=SR*3; hdr(wb,n);
        float lp=0;
        for(int i=0;i<n;i++){
            float t=(float)i/SR;
            lp=lp*.86f+rnd()*.14f;
            float mod=.35f+.65f*fabsf(sinf(6.28f*.38f*t));
            s16(wb,(int16_t)(lp*mod*16000));
        }
    }

    // ── Body shop: hammering + welding buzz ───────────────────
    void makeBodyShop(WavBuf& wb) {
        int n=SR*3; hdr(wb,n);
        for(int i=0;i<n;i++){
            float t=(float)i/SR;
            float ph1=fmodf(t,.55f);
            float ph2=fmodf(t+.28f,1.4f);
            int16_t v=0;
            if(ph1<.05f){ float d=1.f-ph1/.05f; v+=(int16_t)(rnd()*d*22000); }
            if(ph2>1.25f){ float ns=rnd(); float bz=sinf(6.28f*3100.f*t); v+=(int16_t)((ns*.4f+bz*.6f)*5500); }
            s16(wb,v);
        }
    }

    // ── Showroom: gentle C-major chord ───────────────────────
    void makeShowroom(WavBuf& wb) {
        int n=SR*4; hdr(wb,n);
        float env=0,envT=(float)n;
        for(int i=0;i<n;i++){
            float t=(float)i/SR;
            float s=sinf(6.28f*261.6f*t)*.40f
                   +sinf(6.28f*329.6f*t)*.30f
                   +sinf(6.28f*392.0f*t)*.30f;
            float e=sinf(3.14159f*(float)i/envT);
            s16(wb,(int16_t)(s*e*5500));
        }
    }
};
