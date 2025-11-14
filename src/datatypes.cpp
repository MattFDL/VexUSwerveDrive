
#include "datatypes.h"
#define _USE_MATH_DEFINES
#include <math.h>

struct vec2{
    double x,y;
    vec2(double x,double y):x(x),y(y){}
    vec2()=default;
    inline vec2 operator=(vec2 a){
        x=a.x;
        y=a.y;
        return *this;
    }
    inline vec2 operator+(vec2 a){
        x+=a.x;
        y+=a.y;
        return vec2(a.x+x,a.y+y);
    }
    inline vec2 operator-(vec2 a){
        x-=a.x;
        y-=a.y;
        return vec2(x-a.x,y-a.y);
    }
    inline vec2 operator*(double a){
        x*=a;
        y*=a;
        return vec2(x*a,y*a);
    }
    inline vec2 operator/(double a){
        x/=a;
        y/=a;
        return vec2(x/a,y/a);
    }
    inline vec2 operator+=(vec2 a){
        x+=a.x;
        y+=a.y;
        return vec2(a.x+x,a.y+y);
    }
    inline vec2 operator-=(vec2 a){
        x-=a.x;
        y-=a.y;
        return vec2(x-a.x,y-a.x);
    }
    inline vec2 operator*=(double a){
        x*=a;
        y*=a;
        return vec2(x*a,y*a);
    }
    inline vec2 operator/=(double a){
        x/=a;
        y/=a;
        return vec2(x/a,y/a);
    }
    inline bool operator==(vec2 a){
        return ((a.x==x)&&(a.y==y));
    }inline bool operator!=(vec2 a){
        return ((a.x!=x)||(a.y!=y));
    }
    double len(){
        return pow(pow(x,2)+pow(y,2),.5);
    }
    double distanceTo(vec2 a){
        return (a-*this).len();
    }
    double cross(vec2 a){
        //cout<<"x: "<<x<<"y: "<<y<<"a.x: "<<a.x<<"a.y: "<<a.y<<'\n';
        //cout<<(x*a.y)<<'-'<<(y*a.x)<<'='<<(x*a.y)-(y*a.x)<<'\n';
        return((x*a.y)-(y*a.x));
    }
};

bool range(double num,double max,double min){
    if ((num<max)&&(num>min)){
        return true;
    }
    return false;
}
