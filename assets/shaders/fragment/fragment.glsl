#version 460

#define MAX_STEPS 1024
#define MAX_DIST 200.
#define SURF_DIST .01

out vec4 out_color;

//==================================================================
//---------------------- 3D GEOMETRIC ALGEBRA ----------------------

struct kln_motor
{
    vec4 p1;  // (1, yz, zx, xy)
    vec4 p2;  // (I, x, y, z)
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
//---------------------------- OBJECTS -----------------------------

struct Object
{
    kln_motor pose;    // pose.p1 = [1, yz, zx, xy] | pose.p2 = [I, x, y, z]
};

layout (std140) buffer ObjectBuffer {
    Object objects[];
};

kln_motor camera_pose = objects[0].pose;

//==================================================================
//-------------- PRIMATIVE SIGNED DISTANCE FUNCTIONS ---------------

//---------------------------- SPHERE ------------------------------
float sdf_sphere(vec4 pnt, vec4 sphere_center, float radius)
{
    return length(pnt.yzw - sphere_center.yzw) - radius;
}

//------------------------------ BOX -------------------------------
float sdf_box(vec4 pnt, kln_motor inv_pose, vec3 box_dim)
{
    vec3 q = abs(kln_apply(inv_pose, pnt)).yzw - box_dim;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

//==================================================================
//--------------------------- RENDERING ----------------------------

vec2 window_res = vec2(1280, 720);
float fov = 1;

float calculate_distance(vec4 pnt)
{
    vec4 sphere_center = vec4(1.0, 0.0, 0.0, 0.0);
    return sdf_sphere(pnt, sphere_center, 0.2);
}

float ray_cast(vec4 ray_origin, vec4 ray_direction)
{
    vec4 pnt = ray_origin;
    float step_size = calculate_distance(pnt);
    float total_distance = step_size;

    int stp= 0;
    for(; stp<MAX_STEPS && !(total_distance>MAX_DIST || step_size<SURF_DIST); stp++) 
    {
        pnt = ray_origin + ray_direction * total_distance;
        step_size = calculate_distance(pnt);
        total_distance += step_size;
    }

    return total_distance;
}

vec4 get_ray_origin()
{
    return kln_apply(camera_pose);
}

vec4 get_ray_direction()
{
    vec2 uv = (gl_FragCoord.xy - 0.5 * window_res - 0.5) / window_res.y;
    vec4 ray_direction = vec4(0, uv, fov);
    ray_direction = kln_apply(camera_pose, ray_direction);
    return normalize(ray_direction);
}

void main()
{
    vec4 ray_origin = get_ray_origin();
    vec4 ray_direction = get_ray_direction();

	float color = ray_cast(ray_origin, ray_direction) / 10;

    out_color = vec4(vec3(color), 1.0);
}
