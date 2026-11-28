#include <vector>
#include <random>
#include "Arduino.h"
unsigned long simUs = 0; bool simVerbose = false;
SerialT Serial;
#include "Wire.h"
WireT Wire;
#include "EEPROM.h"
EEPROMT EEPROM;
static void simPwmLevel(int pin, unsigned level);
#include "hardware/pwm.h"
#define pwm_set_gpio_level(p,l) simPwmLevel((p),(l))
#include SKETCH
#undef pwm_set_gpio_level

// ---------------- field (firmware frame: origin back-left inner corner, x right, y up-field) ----------------
struct Seg{double ax,ay,bx,by;};
std::vector<Seg> walls, lines;
const double BOX_W = 119.53, T1X = 58.24, T1Y = 89.24; double T2X = 51.1, T2Y = 179.5, AXIS = -62.7*PI/180;
double dseg(const Seg&s,double px,double py){double dx=s.bx-s.ax,dy=s.by-s.ay,L2=dx*dx+dy*dy;double t=L2>0?((px-s.ax)*dx+(py-s.ay)*dy)/L2:0;t=t<0?0:(t>1?1:t);double qx=s.ax+t*dx-px,qy=s.ay+t*dy-py;return sqrt(qx*qx+qy*qy);}
bool blackAt(double px,double py){ if(px<5||px>117||py<5||py>239) return true; for(auto&s:lines) if(dseg(s,px,py)<1.25) return true; return false; }

// ---------------- robot ----------------
double rx,ry,rth, vL=0,vR=0, accL=0,accR=0, thUnwrap=0, gyroRef=0;
int pinState[40]; int enVal[40];
std::mt19937 rng(1); std::normal_distribution<double> nrm(0,1);
long quickCalLeft=-1; unsigned long jerkUntil=0; bool dropped=false; double dropX=0,dropY=0; double dropT=0;
static void simPwmLevel(int, unsigned level){ if(level>=2400 && !dropped){ dropped=true; dropX=rx; dropY=ry; dropT=simUs/1e6; } }
void pinMode(int,int){}
void digitalWrite(int p,int v){ if(p>=0&&p<40) pinState[p]=v; }
void analogWrite(int p,int v){ if(p>=0&&p<40) enVal[p]=v; }
int irPin(int p){ double lat = p==PIN_IR_L? 2.0 : (p==PIN_IR_R? -2.0 : 0.0); double f=7.5;
  double sx=rx+f*cos(rth)-lat*sin(rth), sy=ry+f*sin(rth)+lat*cos(rth); return blackAt(sx,sy)?HIGH:LOW; }
int digitalRead(int p){ simAdvance(1); if(p==PIN_IR_L||p==PIN_IR_R||p==PIN_IR_C) return irPin(p); return 0; }
double rayHit(double px,double py,double a){ double best=1e9; bool ok=false;
  for(auto&w:walls){ double dx=cos(a),dy=sin(a),ex=w.bx-w.ax,ey=w.by-w.ay,den=dx*ey-dy*ex; if(fabs(den)<1e-9)continue;
    double t=((w.ax-px)*ey-(w.ay-py)*ex)/den,u=((w.ax-px)*dy-(w.ay-py)*dx)/den; if(t<=0||u<0||u>1)continue;
    if(t<best){ best=t; double nx=-ey,ny=ex,nl=sqrt(nx*nx+ny*ny); ok = acos(fabs((dx*nx+dy*ny)/nl))*180/PI < 40; } }
  return ok? best : -1; }
double usRead(int echo){ double off, side; double a;
  if(echo==F_ECHO){ off=18; side=0; a=rth; } else if(echo==L_ECHO){ off=0; side=10.8; a=rth+PI/2; } else { off=0; side=-11.3; a=rth-PI/2; }
  double px=rx+off*cos(rth)-side*sin(rth), py=ry+off*sin(rth)+side*cos(rth);
  double best=-1; for(int k=-2;k<=2;k++){ double d=rayHit(px,py,a+k*6*PI/180); if(d>0&&(best<0||d<best)) best=d; }
  if(best<0||best>400) return -1; return best+0.3*nrm(rng); }
unsigned long pulseIn(int pin,int,unsigned long to){ double d=usRead(pin); if(d<2){ simAdvance(to); return 0; }
  unsigned long us=(unsigned long)(d/0.01715); if(us>to){ simAdvance(to); return 0; } simAdvance(us+300); return us; }
int hitCount[16]; int lastHitWall=-1;
bool collide(double x,double y,double th){
  double fx=x+15*cos(th), fy=y+15*sin(th);
  for(size_t i=0;i<walls.size();i++){ auto&w=walls[i]; if(dseg(w,x,y)<9 || dseg(w,fx,fy)<3.5){ lastHitWall=(int)i; return true; } } return false; }
int wheelDir(int a,int b,bool left){ // left: IN3 H IN4 L = fwd ; right: IN1 L IN2 H = fwd
  if(left){ if(a&&!b) return 1; if(!a&&b) return -1; return 0; } else { if(!a&&b) return 1; if(a&&!b) return -1; return 0; } }
void physics(double dt){
  if(simVerbose && (simUs%250000)<1000) printf("  @%.2fs pos (%.1f,%.1f) h %.0f\n", simUs/1e6, rx, ry, fmod(rth*180/PI+720,360));
  double tgt[2]; double vmax[2]={40.0, 40.0/0.86}; double* v[2]={&vL,&vR};
  for(int s=0;s<2;s++){
    int en = s==0? enVal[L_ENB] : enVal[R_ENA];
    double duty=(255-en)/255.0;
    int d = s==0? wheelDir(pinState[L_IN3],pinState[L_IN4],true) : wheelDir(pinState[R_IN1],pinState[R_IN2],false);
    double tau;
    if(d==0){ tgt[s]=0; tau = duty>0.5? 0.015 : 0.12; }
    else { tgt[s]= d*vmax[s]*((duty-0.27)/0.73>0?(duty-0.27)/0.73:0.0); tau=0.07; }
    *v[s] += (tgt[s]-*v[s])*(dt/tau>1?1:dt/tau);
  }
  double dl=vL*dt, dr=vR*dt, d=(dl+dr)/2, dth=(dr-dl)/19.0;
  { static double slip=getenv("SLIP")?atof(getenv("SLIP")):0; if(vL*vR<0) dth*=(1-slip); else dth*=(1-0.3*slip); }
  double nx=rx+d*cos(rth+dth/2), ny=ry+d*sin(rth+dth/2), nth=rth+dth;
  double encL=dl, encR=dr;
  if(collide(nx,ny,nth)){ if(simUs>=jerkUntil) hitCount[lastHitWall]++; jerkUntil=simUs+120000; encL*=0.3; encR*=0.3; vL*=0.9; vR*=0.9; dth=0; }
  else { rx=nx; ry=ny; rth=nth; }
  thUnwrap += dth;
  accL+=encL*TICKS_PER_CM_L; accR+=encR*TICKS_PER_CM_R; long iL=(long)accL, iR=(long)accR; accL-=iL; accR-=iR;
  leftCount+=iL; rightCount+=iR;
  // core 1 IMU emulation
#ifdef GYRO_SKETCH
  if(gyroZeroReq){ gyroRef=thUnwrap; gyroZeroReq=false; }
  if(gyroQuickCalReq){ if(quickCalLeft<0) quickCalLeft=450; if(--quickCalLeft<=0){ gyroRef=thUnwrap; gyroQuickCalReq=false; quickCalLeft=-1; } }
  gyroHeadingMdeg=(int32_t)lround((thUnwrap-gyroRef)*180/PI*1000 + 30*nrm(rng));
  gyroRateMdps=(int32_t)lround(dth/dt*180/PI*1000);
  imuJerkPeakMg = simUs<jerkUntil? 2000 : 150;
  gyroHeartbeat++;
#endif

}
unsigned long physUs = 0;
void simAdvance(unsigned long us){
  simUs += us;
  while(physUs + 1000 <= simUs){ physUs += 1000; physics(0.001);
    if(physUs>130000000UL){ printf("SIM TIMEOUT\n"); fflush(stdout); exit(3);} }
}
int main(int argc,char**argv){
  rx=atof(argv[1]); ry=atof(argv[2]); rth=atof(argv[3])*PI/180; simVerbose = argc>4;
  walls={{0,0,BOX_W,0},{0,0,0,58},{BOX_W,0,BOX_W,58},
    {22.4,190.6,48.7,139.7},{58.5,209.6,85.8,156.8},{19.1,172.3,29.2,177.5},{34.3,167.5,70.6,186.2},{43.8,149.3,80.0,168.0},{88.1,215.8,106.1,181.0},{95.7,201.2,109.3,208.2}};
  double P[][2]={{94.0,114.8},{65.2,123.3},{42.1,125.9},{19.0,122.8},{16.2,108.3},{20.6,91.2},{28.4,78.6},{38.7,68.9},{52.1,63.7},{65.0,63.4},{77.8,66.4},{85.9,73.4},{91.9,83.7},{98.9,101.2},{102.9,118.7},{105.2,134.1},{104.7,146.0},{102.3,157.2},{97.8,164.9},{90.1,177.5},{80.0,198.8},{73.4,214.0},{70.5,224.5},{67.5,228.9},{58.9,229.8},{44.2,224.6},{18.5,213.7},{32.9,209.1},{50.5,202.0}};
  for(int i=0;i<28;i++) lines.push_back({P[i][0],P[i][1],P[i+1][0],P[i+1][1]});
  { double pdx=getenv("PDX")?atof(getenv("PDX")):0, pdy=getenv("PDY")?atof(getenv("PDY")):0, pth=getenv("PTH")?atof(getenv("PTH"))*PI/180:0;
    auto tf=[&](double&x,double&y){ double dx=x-T2X, dy=y-T2Y; x=T2X+dx*cos(pth)-dy*sin(pth)+pdx; y=T2Y+dx*sin(pth)+dy*cos(pth)+pdy; };
    for(int i=3;i<=6;i++){ tf(walls[i].ax,walls[i].ay); tf(walls[i].bx,walls[i].by); }
    T2X+=pdx; T2Y+=pdy; AXIS+=pth; }
  for(int i=0;i<40;i++){pinState[i]=0; enVal[i]=255;}

#ifdef GYRO_SKETCH
  gyroHwOk=true; gyroReady=true;
#endif
  setup();
  { double best=1e9,bi=0; for(int i=0;i<28;i++){ double d=dseg(lines[i],rx,ry); if(d<best){best=d;bi=i;} } printf("LINE TEST: ended at segment %d of 28 (0=dead end of loop, 27=line end F) | ", (int)bi); }
  double tx=rx+18*cos(rth), ty=ry+18*sin(rth);
  double ax=cos(AXIS), ay=sin(AXIS); double lat = -(rx-T2X)*ay + (ry-T2Y)*ax;   // + = left of centre line
  double hErr = fmod((rth-AXIS)*180/PI+540*3,360)-180; int pocketHits=hitCount[3]+hitCount[4]+hitCount[5]+hitCount[6];
  { const char* nm[]={"back","left box","right box","W1","W2","C1","C2","C3","R1","R2"}; printf("hits:"); for(int i=0;i<10;i++) if(hitCount[i]) printf(" %s x%d",nm[i],hitCount[i]); printf(" | "); }
  printf("POCKET lat %+5.1f cm | heading %+5.1f deg | tip-to-T2 along %+5.1f cm | pocket wall hits %d | all hits %d | ",
     lat, hErr, (tx-T2X)*ax+(ty-T2Y)*ay, pocketHits, pocketHits+hitCount[0]+hitCount[1]+hitCount[2]+hitCount[7]+hitCount[8]+hitCount[9]);
  printf("start (%5.1f,%4.1f,%4.0f): T1 drop %s %5.1f cm off at %4.1fs | end (%5.1f,%5.1f) h %4.0f | front tip %5.1f cm from T2 | t %5.1fs\n",
    atof(argv[1]),atof(argv[2]),atof(argv[3]), dropped?"":"NONE", dropped?hypot(dropX-T1X,dropY-T1Y):-1, dropT, rx,ry,fmod(rth*180/PI+720,360), hypot(tx-T2X,ty-T2Y), simUs/1e6);
}
