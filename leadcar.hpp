#ifndef LEADCAR_H
#define LEADCAR_H
#include<vector>

namespace acc{

    // type structure
    typedef struct {    
        size_t step_no;
    }my_sim_step;                                                                                       // will be used to send data into NLOPT function
    
    // structures
    struct point2D {
        double x;
        double y;

        point2D(double n1, double n2) : x(n1), y(n2) {}
        point2D() : x(0.0), y(0.0) {}
        // Optional: Overload << for easy printing
        //friend std::ostream& operator<<(std::ostream& os, const point2D& point) {
        //    os << "(" << point.x << ", " << point.y << ")";
        //    return os;
    };

    // const variables
    static const double controller_timestep (0.05);                                                    // time gap between the current and next MPC trigger (in secs) 
    const double control_horizon_in_secs (0.3);                                                        // control horizon (in secs)
    static const int control_horizon(6);                                                               // optimization variables = no of MPC triggers to control horizon

    // class
    // Lead car  
    // In future - It Will handle LEAD CAR SPEED PROFILE as Input
    class cl_leadCar
    {
        private:
            // variable
            std::vector<double> leadCar_speed_profile {std::vector<double>(12000)};                     // speed profile of the lead vehicle = USER INPUT FOR 10 mins  
            std::vector<acc::point2D> user_inputs;
            double m,c;
            acc::point2D p1,p2;
            int counter;

        public:
            // constructor declaration              
            cl_leadCar();
        protected:
            // function declaration    
            double get_curr_leadCarSpeed(const int &current_timestep);                                  // Get the current user input speed of the lead car                
    };
}
#endif // !LEADCAR_H
