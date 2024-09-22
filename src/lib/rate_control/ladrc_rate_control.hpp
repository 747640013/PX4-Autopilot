/**
 * @file rate_control.hpp
 *
 * LADRC 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class LadrcRateControl
{
private:
	/* TD 滤波器参数*/
	matrix::Vector3f _a0{0.f, 0.f, 0.f};
	matrix::Vector3f _a1{0.f, 0.f, 0.f};

	/* TD 输入输出*/
	matrix::Vector3f v{0.f, 0.f, 0.f};
	matrix::Vector3f v1{0.f, 0.f, 0.f};
	matrix::Vector3f v2{0.f, 0.f, 0.f};

	/* 控制量*/
	matrix::Vector3f u0{0.f, 0.f, 0.f};
	matrix::Vector3f u{0.f, 0.f, 0.f};
	matrix::Vector3f e1{0.f, 0.f, 0.f};
	matrix::Vector3f e2{0.f, 0.f, 0.f};

	/*输出限幅*/
	matrix::Vector3f _umin{0.f, 0.f, 0.f};
	matrix::Vector3f _umax{0.f, 0.f, 0.f};

	/*控制增益*/
	matrix::Vector3f _beat1{0.f, 0.f, 0.f};
	matrix::Vector3f _beat2{0.f, 0.f, 0.f};

	/*扰动增益*/
	matrix::Vector3f _disturb_gain{1.f, 1.f, 1.f};


	/*eso 状态量*/
	matrix::Vector3f y{0.f, 0.f, 0.f};
	matrix::Vector3f z1{0.f, 0.f, 0.f};
	matrix::Vector3f z2{0.f, 0.f, 0.f};
	matrix::Vector3f z3{0.f, 0.f, 0.f};
	matrix::Vector3f disturb{0.f, 0.f, 0.f};

	/*扰动限幅*/
	matrix::Vector3f _disturb_min{0.f, 0.f, 0.f};
	matrix::Vector3f _disturb_max{0.f, 0.f, 0.f};

	/*扰动增益*/
	matrix::Vector3f _b0{0.f, 0.f, 0.f};

	/*观测器参数*/
	matrix::Vector3f _beta01{0.f, 0.f, 0.f};
	matrix::Vector3f _beta02{0.f, 0.f, 0.f};
	matrix::Vector3f _beta03{0.f, 0.f, 0.f};

public:
	LadrcRateControl(/* args */) = default;
	~LadrcRateControl() = default;

	/**
	 * 设置跟踪微分器参数
	 * @param a0 x y z  Wn*Wn
	 * @param a1 x y z  2*zeta*Wn
	 */
	void set_td_coef(const matrix::Vector3f &a0, const matrix::Vector3f &a1);

	void td_update(const matrix::Vector3f &input, const float dt);

	void td_reset();

	matrix::Vector3f acquire_control_input();

	matrix::Vector3f acquire_tracking_signal();

	matrix::Vector3f acquire_differential_signal();


	void set_gains(const matrix::Vector3f &beta1, const matrix::Vector3f &beta2);

	void set_output_limit(const matrix::Vector3f &umin, const matrix::Vector3f &umax);

	void set_distrub_gain(const matrix::Vector3f &gain);

	void ctl_update(const matrix::Vector3f &v1d, const matrix::Vector3f &v2d,
			const matrix::Vector3f &z1, const matrix::Vector3f &z2,
			const matrix::Vector3f &disturb);

	void reset();

	matrix::Vector3f acquire_e1();

	matrix::Vector3f acquire_e2();

	matrix::Vector3f acquire_u0();

	matrix::Vector3f acquire_ouptut();

	void set_eso_coef(const matrix::Vector3f &b0, const matrix::Vector3f &beta01,
				 const matrix::Vector3f &beta02, const matrix::Vector3f &beta03);


	void set_distrub_limit(const matrix::Vector3f &dmin,const matrix::Vector3f &dmax);

        void eso_update(const matrix::Vector3f &u,const matrix::Vector3f &y,const float dt);

        void eso_reset();

	matrix::Vector3f acquire_eso_reference();

	matrix::Vector3f acquire_eso_z1();

	matrix::Vector3f acquire_eso_z2();

	matrix::Vector3f acquire_eso_z3();

	matrix::Vector3f acquire_eso_disturb();

};

