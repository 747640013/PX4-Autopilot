/**
 * @file rate_control.hpp
 *
 * LADRC 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/topics/ladrc_status.h>
#include <drivers/drv_hrt.h>

class LadrcRateControl
{
private:
	/* TD 滤波器参数*/
	matrix::Vector3f _a0{0.f, 0.f, 0.f};
	matrix::Vector3f _a1{0.f, 0.f, 0.f};

	/* TD 状态量*/
	matrix::Vector3f _v{0.f, 0.f, 0.f};
	matrix::Vector3f _v1{0.f, 0.f, 0.f};
	matrix::Vector3f _v2{0.f, 0.f, 0.f};

	/* 控制量*/
	matrix::Vector3f _u0{0.f, 0.f, 0.f};
	matrix::Vector3f _u{0.f, 0.f, 0.f};
	matrix::Vector3f _e1{0.f, 0.f, 0.f};
	matrix::Vector3f _e2{0.f, 0.f, 0.f};

	/*输出限幅*/
	matrix::Vector3f _umin{0.f, 0.f, 0.f};
	matrix::Vector3f _umax{0.f, 0.f, 0.f};

	/*控制增益*/
	matrix::Vector3f _beta1{0.f, 0.f, 0.f};
	matrix::Vector3f _beta2{0.f, 0.f, 0.f};

	/*扰动增益*/
	matrix::Vector3f _disturb_gain{1.f, 1.f, 1.f};

	/*eso 状态量*/
	matrix::Vector3f _y{0.f, 0.f, 0.f};
	matrix::Vector3f _z1{0.f, 0.f, 0.f};
	matrix::Vector3f _z2{0.f, 0.f, 0.f};
	matrix::Vector3f _z3{0.f, 0.f, 0.f};
	matrix::Vector3f _disturb{0.f, 0.f, 0.f};

	/*扰动限幅*/
	matrix::Vector3f _disturb_min{0.f, 0.f, 0.f};
	matrix::Vector3f _disturb_max{0.f, 0.f, 0.f};

	/*扰动增益,注意避免除0*/
	matrix::Vector3f _b0{1e5, 1e5, 1e5};

	/*观测器参数*/
	matrix::Vector3f _beta01{0.f, 0.f, 0.f};
	matrix::Vector3f _beta02{0.f, 0.f, 0.f};
	matrix::Vector3f _beta03{0.f, 0.f, 0.f};

public:
	LadrcRateControl(/* args */) = default;
	~LadrcRateControl() = default;

	/**
	 * 设置跟踪微分器参数
	 * @param a0 三轴，含义：Wn*Wn
	 * @param a1 三轴，含义：2*zeta*Wn
	 */
	void set_td_coef(const matrix::Vector3f &wn, const matrix::Vector3f &zeta);

	/**
	 * 更新跟踪微分器状态量
	 * @param input 三轴，角速度环输入，即角度环输出
	 * @param dt 三轴，时间间隔
	 */
	void td_update(const matrix::Vector3f &input, const float dt);

	/**
	 * 重置跟踪微分器临时变量
	 */
	void td_reset(void);

	matrix::Vector3f acquire_control_input(void);

	matrix::Vector3f acquire_tracking_signal(void);

	matrix::Vector3f acquire_differential_signal(void);


	/**
	 * 设置控制器参数
	 * @param beta1 误差增益
	 * @param beta2 误差导数增益
	 */
	void set_gains(const matrix::Vector3f &beta1, const matrix::Vector3f &beta2);

	void set_output_limit(const matrix::Vector3f &umin, const matrix::Vector3f &umax);

	/**
	 * 设置扰动增益
	 * @param gain 扰动增益
	 */
	void set_distrub_gain(const matrix::Vector3f &gain);

	void ctl_update(const matrix::Vector3f &v1d, const matrix::Vector3f &v2d,
				    const matrix::Vector3f &z1, const matrix::Vector3f &z2,
				    const matrix::Vector3f &disturbance);

	/**
	 * 重置控制量
	 */
	void ctl_reset(void);

	matrix::Vector3f acquire_e1(void);

	matrix::Vector3f acquire_e2(void);

	matrix::Vector3f acquire_u0(void);

	matrix::Vector3f acquire_ouptut(void);

	/**
	 * 设置观测器参数
	 * @param b0 观测器输入增益
	 * @param wc 观测器带宽
	 */
	void set_eso_coef(const matrix::Vector3f &b0, const matrix::Vector3f &wc);

	/**
	 * 设置观测器扰动输出限幅值
	 * @param dmin 下限
	 * @param dmax 上限
	 */
	void set_disturb_limit(const matrix::Vector3f &dmin, const matrix::Vector3f &dmax);

	/**
	 * 观测器更新计算函数
	 * @param u 控制输入
	 * @param y 传感器测量值，角速度
	 * @param dt 时间间隔
	 */
	void eso_update(const matrix::Vector3f &u, const matrix::Vector3f &y, const float dt);

	void eso_reset(void);

	matrix::Vector3f acquire_eso_reference(void);

	matrix::Vector3f acquire_eso_z1(void);

	matrix::Vector3f acquire_eso_z2(void);

	matrix::Vector3f acquire_eso_z3(void);

	matrix::Vector3f acquire_eso_disturb(void);

	/**
	 * 角速度环更新计算函数
	 * @param rate 当前角速度估计值
	 * @param rate_sp 期望角速度
	 * @param dt 时间间隔
	 * @return 控制力距
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp, const float dt,const bool landed);

	void reset(void);

	void clamp( matrix::Vector3f &val, const matrix::Vector3f &min_val,const matrix::Vector3f &max_val);

	void record_adrc_status(ladrc_status_s &rate_status,LadrcRateControl &ladrc);
};
