#ifndef EKF_HPP
#define EKF_HPP

#include "../lib/Eigen/Core"
#include "../lib/Eigen/QR"
#include "../lib/Eigen/Geometry"
#include "../lib/Eigen/LU"
#include <iostream>


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
		MatrixXf Q; 					// Process Noise Covariance
		MatrixXf R; 					// Measurement Noise Covariance
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
	/* State transition function g where
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
		state_dim = 0;
		meas_dim = 0;


	}
	//Constructor
	EKF(int state_dimension, int measurement_dimension)
	{
		state_dim = state_dimension;
		meas_dim = measurement_dimension;

		Q = 0.1*MatrixXf::Identity(state_dim, state_dim);
		R = 0.1* MatrixXf::Identity(meas_dim, meas_dim);

		gravity_vector << 0, 0, 0;
		//cout << gravity_vector << endl;
		//VectorXf initial_estimate(state_dim);
		//initial_estimate << 5, 5, 5, 1, 2, 3;
		//state_estimates.push_back(initial_estimate);
		//cout << state_estimates.front() << endl;

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

	void update_measurement(float new_meas[])
	{
		// TODO: check if new measurement is right dimension
		VectorXf new_meas_vec(meas_dim);
		// copy the new vector over
		// for (int i = 0; i < meas_dim; i++)
		// {
		// 	new_meas_vec(i) = new_meas[i]; 
		// }

		array_to_vectorXf(new_meas, meas_dim, new_meas_vec);
		measurements.push_back(new_meas_vec);
		// cout << "measurement updated: 	";
		// cout << new_meas_vec << endl;
	}

	void update_measurement(VectorXf &new_meas_vec, float meas_dt)
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
		measurements.push_back(new_meas_vec);
		dt = meas_dt;
		// cout << "measurement updated: 	";
		// cout << new_meas_vec << endl;
		VectorXf next_state(6);

		// Predict
		VectorXf state_predict(state_dim);
		state_predict = state_transition(state_estimates.back());


		MatrixXf covariance_predict(state_dim, state_dim);
		covariance_predict = A*covariance_estimates.back()*A.transpose() + Q;

		// update step
		MatrixXf sig_t;
		Vector3f innovation = measurements.back() - measurement_model(state_predict);
		
		MatrixXf KalmanGain(meas_dim, state_dim);
		KalmanGain = covariance_predict* C.transpose() * (C*covariance_predict*C.transpose() + R);

		VectorXf x_estimate(state_dim);
		x_estimate = state_predict + KalmanGain*innovation;
		state_estimates.push_back(x_estimate);

		MatrixXf cov_estimate(state_dim, state_dim);
		cov_estimate = (MatrixXf::Identity(state_dim, state_dim) - KalmanGain*C) * covariance_predict;
		covariance_estimates.push_back(cov_estimate);
		cout << "delta_t: 	" << dt << endl;
		cout << "New estimate: 	" << state_estimates.back() << endl;
	}




};
#endif // EKF_HPP