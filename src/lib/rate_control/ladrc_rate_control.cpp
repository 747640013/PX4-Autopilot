#include<ladrc_rate_control.hpp>

using namespace matrix;


void LadrcRateControl::clamp(const matrix::Vector3f &val, const matrix::Vector3f &min_val,
			     const matrix::Vector3f &max_val)
{
	for (int i = 0; i < 3; ++i) {
		(val(i)) < min_val(i) ? min_val(i) : ((val(i) > max_val(i)) ? max_val(i) : val(i));
	}
}

void LadrcRateControl::set_td_coef(const Vector3f &wn, const Vector3f &zeta)
{
	for (int i = 0; i < 3; ++i) {
		_a0(i) = wn(i) * wn(i);
		_a1(i) = 2.f * wn(i) * zeta(i);
	}

}

void LadrcRateControl::td_update(const Vector3f &input, const float dt)
{
	_v = input;
	_v1 += _v2 * dt;
	_v2 += (_a0.emult(_v - _v1) - _a1.emult(_v2)) * dt;
}

Vector3f LadrcRateControl::acquire_control_input()
{
	return _v;
}

Vector3f LadrcRateControl::acquire_tracking_signal()
{
	return _v1;
}

Vector3f LadrcRateControl::acquire_differential_signal()
{
	return _v2;
}

void LadrcRateControl::td_reset()
{
	_v.setZero();
	_v1.setZero();
	_v2.setZero();
}

void LadrcRateControl::set_gains(const Vector3f &beta1, const Vector3f &beta2)
{
	_beta1 = beta1;
	_beta2 = beta2;
}

void LadrcRateControl::set_output_limit(const Vector3f &umin, const Vector3f &umax)
{
	_umin = umin;
	_umax = umax;
}

void LadrcRateControl::set_distrub_gain(const Vector3f &gain)
{
	_disturb_gain = gain;
}

Vector3f LadrcRateControl::ctl_update(const Vector3f &v1d, const Vector3f &v2d,
				      const Vector3f &z1, const Vector3f &z2,
				      const Vector3f &disturbance)
{
	_e1 = v1d - z1;
	_e2 = v2d - z2;
	_u0 = _beta1.emult(_e1) + _beta2.emult(_e2);
	_u = _u0 + _disturb_gain.emult(disturbance);
	clamp(_u, _umin, _umax);
	return _u;
}

void LadrcRateControl::ctl_reset()
{
	_u0.setZero();
	_u.setZero();
	_e1.setZero();
	_e2.setZero();
}

Vector3f LadrcRateControl::acquire_e1()
{
	return _e1;
}

Vector3f LadrcRateControl::acquire_e2()
{
	return _e2;
}

Vector3f LadrcRateControl::acquire_u0()
{
	return _u0;
}

Vector3f LadrcRateControl::acquire_ouptut()
{
	return _u;
}

void LadrcRateControl::set_eso_coef(const Vector3f &b0, const Vector3f &wc)
{
	_b0 = b0;

	for (int i = 0; i < 3; ++i) {
		_beta01(i) = 3.0f * wc(i);
		_beta02(i) = 3.0f * wc(i) * wc(i);
		_beta03(i) = wc(i) * wc(i) * wc(i);
	}
}


void LadrcRateControl::set_disturb_limit(const Vector3f &dmin,
		const Vector3f &dmax)
{
	_disturb_max = dmin;
	_disturb_min = dmax;
}

void LadrcRateControl::eso_update(const Vector3f &input, const Vector3f &y_, const float dt)
{
	_y = y_;
	Vector3f err = _z1 - _y;
	_z1 += (_z2 - _beta01.emult(err)) * dt;
	_z2 += (_z3 - _beta02.emult(err) + _b0.emult(input)) * dt;
	_z3 += -_beta03.emult(err);

	_disturb = -_z3.edivide(_b0);
	clamp(_disturb, _disturb_min, _disturb_max);
}

void LadrcRateControl::eso_reset()
{
	_z1.setZero();
	_z2.setZero();
	_z3.setZero();
	_y.setZero();
	_disturb.setZero();
}

Vector3f LadrcRateControl::acquire_eso_reference()
{
	return _y;
}

Vector3f LadrcRateControl::acquire_eso_z1()
{
	return _z1;
}

Vector3f LadrcRateControl::acquire_eso_z2()
{
	return _z2;
}
Vector3f LadrcRateControl::acquire_eso_z3()
{
	return _z3;
}

Vector3f LadrcRateControl::acquire_eso_disturb()
{
	return _disturb;
}

Vector3f LadrcRateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const float dt, const bool landed)
{
	td_update(rate_sp, dt);
	Vector3f v1 = acquire_control_input();
	Vector3f v2 = acquire_differential_signal();

	Vector3f z1 = acquire_eso_z1();
	Vector3f z2 = acquire_eso_z2();
	//Vector3f z3_temp = acquire_eso_z3();
	Vector3f disturb = acquire_eso_disturb();

	Vector3f u = ctl_update(v1, v2, z1, z2, disturb);

	eso_update(u, rate, dt);

	if (landed) {
		reset();
	}

	return u;

}

void LadrcRateControl::reset()
{
	td_reset();
	ctl_reset();
	eso_reset();
}

void LadrcRateControl::record_adrc_status(ladrc_status_s &rate_status, LadrcRateControl &ladrc)
{
	rate_status.timestamp = hrt_absolute_time();

	for (int i = 0; i < 3; ++i) {
		rate_status.v[i] = ladrc.acquire_control_input()(i);
		rate_status.v1[i] = ladrc.acquire_tracking_signal()(i);
		rate_status.v2[i] = ladrc.acquire_differential_signal()(i);

		rate_status.e1[i] = ladrc.acquire_e1()(i);
		rate_status.e2[i] = ladrc.acquire_e2()(i);
		rate_status.u0[i] = ladrc.acquire_u0()(i);
		rate_status.u[i] = ladrc.acquire_ouptut()(i);

		rate_status.y[i] = ladrc.acquire_eso_reference()(i);
		rate_status.z1[i] = ladrc.acquire_eso_z1()(i);
		rate_status.z2[i] = ladrc.acquire_eso_z2()(i);
		rate_status.z3[i] = ladrc.acquire_eso_z3()(i);
		rate_status.disturb[i] = ladrc.acquire_eso_disturb()(i);
	}
}
