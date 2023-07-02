#version 460

#define MAX_STEPS 1024
#define MAX_DIST 200.
#define SURF_DIST .01

flat in vec3 Color;
out vec4 outColor;
uniform vec4 tex;

//==================================================================
//---------------------- 3D GEOMETRIC ALGEBRA ----------------------

struct kln_motor
{
    vec4 p1;
    vec4 p2;
};

vec4 kln_apply(in kln_motor m)
{
    vec4 p;
    p = m.p1 * m.p2.x;
    p += m.p1.x * m.p2;
    p += m.p1.xwyz * m.p2.xzwy;
    p = m.p1.xzwy * m.p2.xwyz - p;
    p *= vec4(0.0, 2.0, 2.0, 2.0);
    p.x = 1.0;
    return p;
}

vec4 kln_apply(in kln_motor m, in vec4 p)
{
    vec4 scale = vec4(0, 2, 2, 2);

    vec4 t1 = m.p1 * m.p1.xwyz;
    t1 -= m.p1.x * m.p1.xzwy;
    t1 *= scale;

    vec4 t2 = m.p1.x * m.p1.xwyz;
    t2 += m.p1.xzwy * m.p1;
    t2 *= scale;

    vec4 t3 = m.p1 * m.p1;
    t3 += m.p1.yxxx * m.p1.yxxx;
    vec4 t4 = m.p1.zwyz * m.p1.zwyz;
    t4 += m.p1.wzwy * m.p1.wzwy;
    t3 -= t4 * vec4(-1.0, 1.0, 1.0, 1.0);

    t4 = m.p1.xzwy * m.p2.xwyz;
    t4 -= m.p1.x * m.p2;
    t4 -= m.p1.xwyz * m.p2.xzwy;
    t4 -= m.p1 * m.p2.x;
    t4 *= scale;

    vec4 q;
    q = t1 * p.xwyz;
    q += t2 * p.xzwy;
    q += t3 * p;
    q += t4 * p.x;
    return  q;
}

//==================================================================
//----------------------------- CAMERA -----------------------------

layout (std140) uniform Camera 
{
    vec4 c1;
    vec4 c2;
};

kln_motor camPose = kln_motor(vec4(-1, 0, 0, 0), vec4(0, 1, -3, 0));
vec2 windowRes = vec2(1280, 720);
float fov = 2;
float time = 0.0;
kln_motor tempCube = kln_motor(vec4(-1, 1, 1, 0), vec4(0, 1, -3, 0));
//==================================================================
//---------------------------- OBJECTS -----------------------------

struct Object
{
    vec4 position;
    vec4 color;
};

layout (std140) buffer ObjectBuffer {
    Object objects[];
};

//==================================================================
//-------------- PRIMATIVE SIGNED DISTANCE FUNCTIONS ---------------


//---------------------------- SPHERE ------------------------------

float sdSphere(vec4 pnt, vec4 sphereCenter, float radius)
{
    return length(pnt.yzw - sphereCenter.yzw) - radius;
}


//------------------------------ BOX -------------------------------

float sdBox(vec4 pnt, kln_motor invPose, vec3 boxDim)
{
    vec3 q = abs(kln_apply(invPose, pnt)).yzw - boxDim;
    return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}


//==================================================================
//-------------- PRIMATIVE SIGNED LIGHTING FUNCTIONS ---------------


//------------------------- SPHERE LIGHT ---------------------------

float slSphere(vec4 pnt, vec4 sphereCenter, float radius)
{
    return pow(length(pnt.wyz - sphereCenter.wyz) - radius, -2);
}

//--------------------- IDEAL/INF SPHERE LIGHT ---------------------


//==================================================================
//---------------------- RENDERING FUNCTIONS -----------------------

struct distSteps
{
    float dist;
    int steps;
};

float calcDist(vec4 pnt)
{
    float sphereDist = sdSphere(pnt, vec4(1, 1, 1, 6), 1.0);
    float cubeDist = sdBox(pnt, tempCube, vec3(1.0,1.5,0.5));
    float planeDist = pnt.z;

    float d = min(sphereDist,min( planeDist, cubeDist));
    return d;
}

distSteps rayMarch(vec4 rayOrigin, vec4 rayDir)
{
    vec4 pnt = rayOrigin;
    float distStep = calcDist(pnt);
    float distTotal = distStep;

    int stp= 0;
    for(; stp<MAX_STEPS && !(distTotal>MAX_DIST || distStep<SURF_DIST); stp++) 
    {
        pnt = rayOrigin + rayDir*distTotal;
        distStep = calcDist(pnt);
        distTotal += distStep;
    }

    distSteps ds;
    ds.dist = distTotal;
    ds.steps = stp;
    return ds;
}

vec4 calcNormal(vec4 pnt, float h)
{
    #define ZERO (min(int(windowRes.x),0)) // non-constant zero
    vec3 n = vec3(0.0);
    for( int i=ZERO; i<4; i++ )
    {
        vec3 e = 0.5773*(2.0*vec3((((i+3)>>1)&1),((i>>1)&1),(i&1))-1.0);
        n += e*calcDist(pnt + vec4(0.0, e*h)).x;
    }
    return vec4(0.0,normalize(n));
}

vec4 calcLighting(vec4 pnt)
{
    vec4 lightPos = vec4(1, 4, 5, 2);
    vec4 lightDir = normalize(lightPos-pnt);
    vec4 normal = calcNormal(pnt, 0.0001);

    float dif = clamp(dot(normal, lightDir), 0., 1.);
    distSteps ds = rayMarch(pnt + normal*SURF_DIST*2., lightDir);
    if(ds.dist<length(lightPos-pnt)) 
        dif *= .1;
    
    return vec4(dif, dif, dif, 1.);
}

vec4 calcFragOrigin()
{
    return kln_apply(camPose);
}

vec4 calcFragDir()
{
    vec2 uv = (gl_FragCoord.xy - 0.5*windowRes - 0.5) / windowRes.y;
    vec4 rayDir = vec4(0, uv, fov);
    rayDir = kln_apply(camPose, rayDir);
    return normalize(rayDir);
}


//==================================================================
//------------------------------ MAIN ------------------------------

void main() 
{
    vec4 rayOrigin = calcFragOrigin();
    vec4 rayDir = calcFragDir();

    distSteps DS = rayMarch(rayOrigin, rayDir);
    float dist = DS.dist;
    int steps = DS.steps;

    vec4 color = tex; //vec4(1., 0.4, 0.0, 1.0);

    if (steps >= MAX_STEPS)
    {
        color = vec4(0.0, 0.8, 1.0, 1.0);
    }
    else if (dist >= MAX_DIST)
    {
        color = vec4(0.8, 0.1, 0.5, 1.0);
    }
    else
    {
	    vec4 p = rayOrigin + rayDir * dist;
        color = calcLighting(p);
    }
    
    color = pow(color, vec4(.4545));	// gamma correction

	outColor = objects[1].color;
}