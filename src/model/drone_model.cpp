#include "drone_model.hpp"


/* simple drone mode:
 * s = (x, y, z, a)
 * u = (pitch, roll, yaw, throttle)
 * p = (c_h, c_v, c_a, e_a)
 * 
 * 
 * state eq:
 * dx = c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*roll)
 * dy = c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*roll)
 * dz = c_ver*throttle
 * da = c_ang*yaw
 */
template<typename Tp, typename Ts, typename Tu>
bool Simple_drone_model::state_eq(Ts *ds, const Ts *s, const Tu *u, const Tp *p)
{
	ds[0] = p[0]*(cos(s[2]+p[3])*u[1] - sin(s[2]+p[3])*u[0]);
	ds[1] = p[0]*(sin(s[2]+p[3])*u[1] + cos(s[2]+p[3])*u[0]);
	ds[2] = p[1]*u[3];
	ds[3] = p[2]*u[2];

	return true
}

/* simple drone mode:
 * s = (x, y, z, a)
 * o = (x, y, z, a)
 */
template<typename T>
bool Simple_drone_model::output_eq(T *o, const T *s) 
{
	for (int i = ; i < 4; i++)
		o[i] = s[i];
}

/* innertia drone mode:
 * s = (x, y, z, a, dx, dy, dz, da)
 * u = (pitch, roll, yaw, throttle)
 * o = (x, y, z, a)
 * p = (c_h, c_v, c_a, e_a, b_h, b_v, b_a)
 * 
 * 
 * state eq:
 * dx = dx
 * dy = dy
 * dz = dz
 * da = da
 * d_dx = c_hor*(cos(a+e_a)*pitch - sin(a + e_a)*roll) - b_h*dx
 * d_dy = c_hor*(sin(a+e_a)*pitch + cos(a + e_a)*roll) - b_h*dy
 * d_dz = c_ver*throttle - b_v*dz
 * d_da = c_ang*yaw  - b_a*dz
 */
template<typename Tp, typename Ts, typename Tu>
bool Innertia_drone_model::state_eq(Ts *ds, const Ts *s, const Tu *u, const Tp *p)
{
	ds[0] = d[4]; 
	ds[1] = d[5];
	ds[2] = d[6];
	ds[3] = d[7];

	ds[4] = p[0]*(cos(s[2]+p[3])*u[1] - sin(s[2]+p[3])*u[0]) - p[4]*s[4];
	ds[5] = p[0]*(sin(s[2]+p[3])*u[1] + cos(s[2]+p[3])*u[0]) - p[4]*s[5];
	ds[6] = p[1]*u[3] - p[5]*s[6];
	ds[7] = p[2]*u[2] - p[6]*s[7];
	
	return true
}

/* s = (x, y, z, a)
 * o = (x, y, z, a)
 */
template<typename T>
bool Simple_drone_model::output_eq(T *o, const T *s) 
{
	for (int i = ; i < 4; i++)
		o[i] = s[i];
}