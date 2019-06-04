#ifndef EKF_HPP
#define EKF_HPP

// #include "../lib/Eigen/Core"
// #include "../lib/Eigen/QR"
// #include "../lib/Eigen/Geometry"
// #include "../lib/Eigen/LU"
#include <iostream>

#define grav_const 9.8
using namespace std;
using namespace Eigen;

class EKF
{	
		
	//Data Members
	private:
		int state_dim;					// Dimension of state vector
		int meas_dim;					// Dimension of measurement vector
		vector<VectorXf> measurements;
		vector<VectorXf> state_estimates;
		MatrixXf Q;		// Process Noise Covariance
		MatrixXf R; 		// Measurement Noise Covariance
		float dt = .0336;						// time between measurements s
		Vector3f gravity_vector;
		MatrixXf A;
		MatrixXf C;
		vector<MatrixXf> covariance_estimates; 


	void array_to_vectorXf(float src[], int dim, VectorXf &dest)
	{
		for (int i = 0; i < dim; i++)
		{
           dest(i) = src[i];
		}
	}

	/* State transition function fx where
	x_t+1 = fx + w_t
	x(0) = px + vx*dt ...
	x(3) = vx + gx*dt ...
	*/

	VectorXf state_transition(VectorXf state_vec)
	{
		VectorXf next_state(state_dim);
		next_state.segment(0,3) = state_vec.segment(0,3) + dt*state_vec.segment(3,3);
		next_state.segment(3,3) = state_vec.segment(3,3) + dt*gravity_vector;
		
		// cout << "next state: 	";
		// cout << next_state << endl;

		return next_state;

	}
	/* measurement transition function g where
	y_t+1 = g(x_t) + v_t
	*/
	VectorXf measurement_model(VectorXf state_vec)
	{
		VectorXf next_measurement(meas_dim);
		next_measurement(0) = state_vec(0);
		next_measurement(1) = state_vec(1);
		next_measurement(2) = state_vec(2);

		return next_measurement;

	}
	// Access specifier
	public:
	EKF()
	{
		state_dim = 6;
		meas_dim = 3;
	}
	//Constructor
	EKF(int state_dimension, int measurement_dimension)
	{
		state_dim = state_dimension;
		meas_dim = measurement_dimension;

		float process_cov_x = 0.0;//0.0596; 
		float process_cov_y = 0.0;//0.0445; 
		float process_cov_z = 0.0;//0.05;

		float process_cov_vx = 0.10;
		float process_cov_vy = 0.15;
		float process_cov_vz = 0.12;

		float meas_cov_x = 100000;//1.1081418750777329e-05 * 10000;
		float meas_cov_y = 100000;//3.3964305601998783e-06 * 10000;
		float meas_cov_z = 100000;//5.551281185489909e-05 * 10000;

		Q = MatrixXf::Identity(state_dim, state_dim);
		Q(0,0) = process_cov_x; 
		Q(1,1) = process_cov_y; 
		Q(2,2) = process_cov_z; 
		
		R = MatrixXf::Identity(meas_dim, meas_dim);
		R(0,0) = meas_cov_x; 
		R(1,1) = meas_cov_y; 
		R(2,2) = meas_cov_z; 


		gravity_vector << 0, 0, 0; 
		// gravity_vector << 0, 0, 0;
		//cout << gravity_vector << endl;
		//VectorXf initial_estimate(state_dim);
		//initial_estimate << 5, 5, 5, 1, 2, 3;
		//state_estimates.push_back(initial_estimate);
		//cout << state_estimates.front() << endl;

		// Calculate A where A = df_i/dx_i
		A = MatrixXf::Identity(state_dim, state_dim);

		A(0, 3) = dt;
		A(1, 4) = dt;
		A(2, 5) = dt;

		C = MatrixXf::Zero(meas_dim, state_dim);
		C(0,0) = 1;
		C(1,1) = 1;
		C(2,2) = 1;

		MatrixXf initial_covariance(state_dim, state_dim);
		initial_covariance = MatrixXf::Identity(state_dim, state_dim);

		covariance_estimates.push_back(initial_covariance);

	}

	// EKF(int state_dimension, int measurement_dimension, MatrixXf Q_set, MatrixXf R_set)
	// {
	// 	state_dim = state_dimension;
	// 	meas_dim = measurement_dimension;
	// 	Q = Q_set;
	// 	R = R_set;
	// 	gravity_vector << 0, 0, -9.8;


	// }
	// EKF(int state_dimension, int measurement_dimension, int Q_set, int R_set)
	// {
	// 	state_dim = state_dimension;
	// 	meas_dim = measurement_dimension;
	// 	Q = MatrixXf::Identity(state_dim, state_dim)*Q_set;
	// 	R = MatrixXf::Identity(meas_dim, meas_dim)*R_set;
	// 	gravity_vector << 0, 0, -9.8;


	// }

	// EKF(int state_dimension, int measurement_dimension, int Q_set, int R_set, float initial_estimate[])
	// {
	// 	// todo check if measurement is right dimension
	// 	state_dim = state_dimension;
	// 	meas_dim = measurement_dimension;
	// 	Q = MatrixXf::Identity(state_dim, state_dim)*Q_set;
	// 	R = MatrixXf::Identity(meas_dim, meas_dim)*R_set;
	// 	gravity_vector << 0, 0, -9.8;

	// }


	//Deconstructor
	~EKF()
	{
		// do something?
	}


	VectorXf getLastEstimate()
	{
		return state_estimates.back();
	}

	VectorXf getInitialEstimate()
	{
		return state_estimates.front();
	}

	
	// void update_measurement(float new_meas[])
	// {
	// 	// TODO: check if new measurement is right dimension
	// 	VectorXf new_meas_vec(meas_dim);
	// 	// copy the new vector over
	// 	// for (int i = 0; i < meas_dim; i++)
	// 	// {
	// 	// 	new_meas_vec(i) = new_meas[i]; 
	// 	// }

	// 	array_to_vectorXf(new_meas, meas_dim, new_meas_vec);
	// 	measurements.push_back(new_meas_vec);
	// 	// cout << "measurement updated: 	";
	// 	// cout << new_meas_vec << endl;
	// }

	void update_measurement(Vector3f new_meas_vec, float meas_dt)
	{

		if (state_estimates.size() == 0)
		{
			cout << "Initial guess" << endl;
			float px = new_meas_vec(0);
			float py = new_meas_vec(1);
			float pz = new_meas_vec(2);

			VectorXf initial_estimate(state_dim); 
			initial_estimate << px, py, pz, 0, 0, 0;

			state_estimates.push_back(initial_estimate);
			cout << state_estimates.back() << endl;
		}
		else
		{
			measurements.push_back(new_meas_vec);
			dt = meas_dt;
			// cout << "measurement updated: 	";
			// cout << new_meas_vec << endl;
			VectorXf next_state(6);

			// Predict
			VectorXf state_predict(state_dim);
			// mu_t|t-1 = f(mu_t-1, u_t-1)
			state_predict = state_transition(state_estimates.back());

			// sigma_t|t-1 = A*sigma_t-1|t-1*A' + Q
			MatrixXf covariance_predict(state_dim, state_dim);
			covariance_predict = A*covariance_estimates.back()*A.transpose() + Q;

			// update step
			
			Vector3f innovation = measurements.back() - measurement_model(state_predict);
			
			MatrixXf KalmanGain(state_dim, meas_dim);
			KalmanGain = covariance_predict* C.transpose() * (C*covariance_predict*C.transpose() + R).inverse();

			VectorXf x_estimate(state_dim);	
			x_estimate = state_predict + KalmanGain*innovation;
			state_estimates.push_back(x_estimate);

			MatrixXf cov_estimate(state_dim, state_dim);
			cov_estimate = (MatrixXf::Identity(state_dim, state_dim) - KalmanGain*C) * covariance_predict;
			
			covariance_estimates.push_back(cov_estimate);
		}
		//cout << "delta_t: 	" << dt << endl;
		// for (int i = 0; i < 3; i++)
		// {
		// 	cout << "New estimate: 	" << x_estimate(i);
		// }
		// cout << endl;

	}

	// void kalman_predictor(double num_tsteps, VectorXf* state_prediction, MatrixXf cov_prediction)
	// {
	// 	VectorXf state_prediction(state_dim);
	// 	MatrixXf cov_prediction(state_dim, state_dim);
		
	// 	state_prediction = state_estimates.back();
	// 	cov_prediction = covariance_estimates.back();

	// 	for (int i = 0:num_tsteps-1)
	// 	{
	// 		state_prediction = A*state_prediction; 
	// 		cov_prediction = A*cov_prediction*A.transpose() + Q;

	// 	}

	// }
	// // /* solve projectile motion equations to predict the amount of time to 
	// reach a certain height, use this height to determine x and z locations
	// at this height
	// */
	// double projectile(double final_height)
	// {
	// 	Vector Xf current_state_vec(state_dim);
	// 	current_state_vec = state_estimates.back();

	// 	float x0 = current_state_vec[0];
	// 	float y0 = current_state_vec[1];
	// 	float z0 = current_state_vec[2];

	// 	float v0x = current_state_vec[3];
	// 	float v0y = current_state_vec[4];
	// 	float v0z = current_state_vec[5];

	// 	float time_to_height;

	// 	disc = pow(v0y,2) - 2*(grav_const) * (y0 - final_height);

	// 	if (disc > 0)
	// 	{
	// 		time_to_height = (-v0y^2 + sqrt(disc))/(2*a);
	// 		return time_to_height;
	// 	}
	// }




};
#endif // EKF_HPP