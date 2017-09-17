#ifndef TWIDDLE_OPTIMIZER_H
#define TWIDDLE_OPTIMIZER_H

template <int N> 
class TwiddleOptimizer {
	public:
	double best_[N];
	double p_[N];
	double dp_[N];
	double best_error_;
	int state_;

	TwiddleOptimizer(double p0 = 0., double dp0 = 1.) :
		best_error_(std::numeric_limits<double>::max()),
   		state_(-1) {
		for(int i=0; i<N; ++i) {
			best_[i] = p0;
			p_[i] = p0;
			dp_[i] = dp0;
		}
	}

	int current_index( void ) { return (state_/2) % N; }

	/** get the current probe value */
	void get( std::array<double, N> & P) {
		for(int i=0; i<N; ++i)
		{
			P[i] = p_[i];
		}
	}
	/** get the current probe value */
	double getBest( std::array<double, N> & P) {
		for(int i=0; i<N; ++i)
		{
			P[i] = best_[i];
		}
		return best_error_;
	}

	/** update the optimizer */
	void update( double error ) {
		// initialize error
		if(state_ == -1)
		{
			best_error_ = error;
			p_[0] += dp_[0];
			++state_;
		}
		// positive direction
		else if(state_ % 2 == 0)
		{
			if(error < best_error_)
			{
				// update best error and parameter step
				best_error_ = error;
				best_[current_index()] = p_[current_index()];
				dp_[current_index()] *= 1.1;

				// skip negative step and optimize next parameter;
				state_+=2;
				p_[current_index()] += dp_[current_index()];
			}
			else
			{
				// try step in negative direction
				++state_;
				p_[current_index()] -= 2*dp_[current_index()];
			}
		}
		// negative direction
		else
		{
			if(error < best_error_)
			{
				// update best error and parameter step
				best_error_ = error;
				dp_[current_index()] *= 1.1;
				best_[current_index()] = p_[current_index()];

			}
			else
			{
				// no improvement, revert change and try smaller step size
				p_[current_index()] += dp_[current_index()];
				dp_[current_index()]  *= 0.9;
			}

			// optimize next parameter
			++state_;
			p_[current_index()] += dp_[current_index()];
		}
	}
};

#endif /* TWIDDLE_OPTIMIZER_H */
