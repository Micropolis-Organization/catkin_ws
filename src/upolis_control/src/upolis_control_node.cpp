#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <dlib/all/source.cpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include "upolis_control/vehicle_model.h"
#include "upolis_control/matplotlibcpp.h"


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>


#define LOOKAHEAD_DISTANCE_M 3
#define END_DIST_THRESHOLD 0.5



using namespace std;
using namespace std::chrono;
namespace plt = matplotlibcpp;

typedef dlib::matrix<double, 0, 1> column_vector;

double *currCoeff;

class uPolisControl {
public:
  typedef struct MPC_WEIGHTS {
      double cte_W,
            eth_W,
            v_W,
            st_rate_W,
            acc_rate_W,
            st_W,
            acc_W;
      MPC_WEIGHTS() {
        this->cte_W = 2700;
        this->eth_W = 2000;
        this->v_W = 1000;
        this->st_rate_W = 600;
        this->acc_rate_W = 200;
        this->st_W = 100;
        this->acc_W = 100;
      }
      MPC_WEIGHTS(double cte_W, double eth_W, double v_W, double st_rate_W, double acc_rate_W, double st_W, double acc_W) {
        this->cte_W = cte_W;
        this->eth_W = eth_W;
        this->v_W = v_W;
        this->st_rate_W = st_rate_W;
        this->acc_rate_W = acc_rate_W;
        this->st_W = st_W;
        this->acc_W = acc_W;
      }
  } MPC_WEIGHTS;

  void step();
  double objective(const column_vector& m);
  void printResults();
  void setVref(double vRef);
  void setCoffs(std::vector<double> &coffs);
  void updateState(double x, double y, double th, double v);
  std::pair<double, double> getCommands();
  vector<double> getCurrPos();
  double getCurrYaw();
  uPolisControl(int N, double dt, double baseLength, MPC_WEIGHTS mpc_w, std::pair<double, double> steerLimits, std::pair<double, double> accLimits);
  ~uPolisControl();

private:
  /* number of horizen point */
  int N; 

  /* MPC cost function weights */
  MPC_WEIGHTS mpc_w;

  /* refernce velocity */
  double v_ref;

  /* sample time */
  double dt;

  /* Base length */
  double baseLength;

  double currSteer;

  /* States */
  state last_state_g;
  state current_state_g;

  /* vector for X, Y, CTE, T for later analysis */
  std::vector<double> X;
  std::vector<double> Y;
  std::vector<double> CTE;
  std::vector<double> T;

  /* coff of refernce polynomial */
  std::vector<double> coff;

  /* Actuation commands output */
  inputs act;

  /* init x solution */
  column_vector x;

  /* Upper bounds */
  column_vector ub;

  /* lower bounds */
  column_vector lb;

  std::pair<double, double> steerLimits, accLimits;

};

uPolisControl::uPolisControl(int N, double dt, double baseLength, MPC_WEIGHTS mpc_w, std::pair<double, double> steerLimits, std::pair<double, double> accLimits) :
  N(N),
  dt(dt),
  baseLength(baseLength),
  mpc_w(mpc_w),
  steerLimits(steerLimits),
  accLimits(accLimits) {
  
  this->coff = { 0, 0,  0, 0 };
  this->v_ref = 1;
  this->currSteer = 0;

  /* init x solution */
  std::vector<double> xVec(2 * this->N, 0);
  this->x = dlib::mat(xVec);

  /* Upper bounds */
  std::vector<double> ubSteer(this->N, this->steerLimits.second),
                      ubAcc(this->N, this->accLimits.second);
  ubSteer.insert(ubSteer.end(), ubAcc.begin(), ubAcc.end());
  this->ub = dlib::mat(ubSteer);

  /* lower bounds */
  std::vector<double> lbSteer(this->N, this->steerLimits.first),
                      lbAcc(this->N, this->accLimits.first);
  lbSteer.insert(lbSteer.end(), lbAcc.begin(), lbAcc.end());
  this->lb = dlib::mat(lbSteer);

  this->act.accelartion = 0;
  this->act.steerangle = 0;
  
}

uPolisControl::~uPolisControl() {
}

double uPolisControl::objective(const column_vector &m) {
    inputs u;
    state last_state = this->current_state_g;
    state current_state = this->current_state_g;
    double Error = 0;
    for (int i = 0; i < this->N; i++)
    {
        u.steerangle = m(i);
        u.accelartion = m(i + N);
        current_state = update_state(u, last_state, &this->coff[0], this->dt, this->baseLength);
        if (i == 0)
        {
            Error += this->mpc_w.cte_W * pow(current_state.cte, 2) + this->mpc_w.eth_W * pow(current_state.eth, 2) + this->mpc_w.v_W * pow((v_ref - current_state.v), 2)
                + this->mpc_w.st_W * pow(u.steerangle, 2) + this->mpc_w.acc_W * pow(u.accelartion, 2);
        }
        else
        {
            Error += this->mpc_w.cte_W * pow(current_state.cte, 2) + this->mpc_w.eth_W * pow(current_state.eth, 2) + this->mpc_w.v_W * pow((v_ref - current_state.v), 2)
                + this->mpc_w.st_rate_W * pow(u.steerangle - m(i - 1), 2) + this->mpc_w.acc_rate_W * pow(u.accelartion - m(i + N - i), 2)
                + this->mpc_w.st_W * pow(u.steerangle, 2) + this->mpc_w.acc_W * pow(u.accelartion, 2);
        }


        last_state = current_state;
    }
    return Error;
}

void uPolisControl::printResults() {

  for (auto i = 0; i < this->X.size(); ++i)
      cout << this->X[i] << " " << this->Y[i] << " ";

  cout << "\n------------------------------------------------------------------------------------------------------" << endl;

}

void uPolisControl::step() {
  /* save start time of optamization */
  auto start = high_resolution_clock::now();

  /* MPC local constrained optamization, the final solutions will be in "column_vector x" */
  auto ptr_to_objective = std::bind(&uPolisControl::objective, this, std::placeholders::_1);
  dlib::find_min_box_constrained(dlib::lbfgs_search_strategy(10),
                            dlib::objective_delta_stop_strategy(1e-11),
                            ptr_to_objective, dlib::derivative(ptr_to_objective), this->x, this->lb, this->ub);

  /* save end time of optamization */
  auto stop = high_resolution_clock::now();

  /* time of optamization */
  auto duration = duration_cast<milliseconds>(stop - start);

  /* Applay output command inputs from optamization to the noisy model of vehicle */
  this->act.steerangle = this->x(0);
  this->act.accelartion = this->x(N);
  // this->current_state_g = update_state(this->act, this->last_state_g, &this->coff[0], this->dt, this->baseLength);


  this->current_state_g.print_state();
  cout << "Time = " << duration.count() << " MS" << endl;
  cout << "Cost = " << objective(x) << endl;

  /* save data for later analysis */
  // this->X.push_back(this->current_state_g.x);
  // this->Y.push_back(this->current_state_g.y);
  this->last_state_g = this->current_state_g;
}

void uPolisControl::setVref(double vRef) {
    this->v_ref = vRef;
}

void uPolisControl::setCoffs(std::vector<double> &coffs) {
  this->coff = coffs;
}

void uPolisControl::updateState(double x, double y, double th, double v) {
  this->last_state_g = this->current_state_g;
  this->current_state_g.x = x;
  this->current_state_g.y = y;
  this->current_state_g.th = th;
  this->current_state_g.v = v;
  double th_des = atan(this->coff[2] + 2 * this->coff[1] * x + 3 * this->coff[0] * pow(x, 2));

  this->current_state_g.cte = polyval(&this->coff[0], 4, x) - y + (v * sin(this->current_state_g.eth) * this->dt);
	this->current_state_g.eth = th - th_des + ((v / 2) * this->act.steerangle * this->dt);

}

std::pair<double, double> uPolisControl::getCommands() {
  std::pair<double, double> ret;

  ROS_INFO("Acc: %f, Steer: %f", this->act.accelartion, this->act.steerangle);
	ret.first = this->current_state_g.v + this->act.accelartion * this->dt;
  ret.second = this->act.steerangle;
  this->currSteer = ret.second;

  // ret.second = this->act.steerangle;
  // ret.first =  this->act.accelartion;

  return ret;
}

vector<double> uPolisControl::getCurrPos() {
  return {this->current_state_g.x, this->current_state_g.y};
}

double uPolisControl::getCurrYaw() {
  return this->current_state_g.th;
}

uPolisControl *uController;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double vel = sqrt(
    msg->twist.twist.linear.x * msg->twist.twist.linear.x
    + msg->twist.twist.linear.y * msg->twist.twist.linear.y
    + msg->twist.twist.linear.z * msg->twist.twist.linear.z
  );

  uController->updateState(msg->pose.pose.position.x,msg->pose.pose.position.y, yaw, vel);

}

template <typename T>
std::vector<T> polyfit_boost(const std::vector<T> &xValues, const std::vector<T> &yValues, const int degree, const std::vector<T>& weights = std::vector<T>())
{
    using namespace boost::numeric::ublas;
    
    if (xValues.size() != yValues.size())
        throw std::invalid_argument("X and Y vector sizes do not match");
    
    bool useWeights = weights.size() > 0 && weights.size() == xValues.size();
    
    // one more because of c0 coefficient
    int numCoefficients = degree + 1;
    
    size_t nCount = xValues.size();
    matrix<T> X(nCount, numCoefficients);
    matrix<T> Y(nCount, 1);
    
    // fill Y matrix
    for (size_t i = 0; i < nCount; i++)
    {
        if (useWeights)
            Y(i, 0) = yValues[i] * weights[i];
        else
            Y(i, 0) = yValues[i];
    }
    
    // fill X matrix (Vandermonde matrix)
    for (size_t nRow = 0; nRow < nCount; nRow++)
    {
        T nVal = 1.0f;
        for (int nCol = 0; nCol < numCoefficients; nCol++)
        {
            if (useWeights)
                X(nRow, nCol) = nVal * weights[nRow];
            else
                X(nRow, nCol) = nVal;
            nVal *= xValues[nRow];
        }
    }
    
    // transpose X matrix
    matrix<T> Xt(trans(X));
    // multiply transposed X matrix with X matrix
    matrix<T> XtX(prec_prod(Xt, X));
    // multiply transposed X matrix with Y matrix
    matrix<T> XtY(prec_prod(Xt, Y));
    
    // lu decomposition
    permutation_matrix<int> pert(XtX.size1());
    const std::size_t singular = lu_factorize(XtX, pert);
    // must be singular
    assert(singular == 0);
    
    // backsubstitution
    lu_substitute(XtX, pert, XtY);
    
    // copy the result to coeff
    return std::vector<T>(XtY.data().begin(), XtY.data().end());
}


static void err_exit(int line, int field, const char *tag, string &s)
{
    cerr << "Format error: line " << line << ", field " << field << ": " << tag << '\n';
    cerr << "Data: " << s << endl;
    exit(1);
}


double calc_dist(std::vector<double> &lhs, std::vector<double> &rhs) {
  return std::sqrt(std::pow(lhs[0] - rhs[0] , 2) + std::pow(lhs[1] - rhs[1], 2));
}

std::vector<double> get_trajectory_coeffs(std::vector<std::vector<double> > &wps, std::vector<double> &wps_dist, std::vector<double> pos, double lookaheadDist = LOOKAHEAD_DISTANCE_M) {

  int minIdx = 0;
  double minDis = INT_MAX;
  for(int i = 0; i < wps.size(); i++) {
    double currDis = calc_dist(wps[i], pos);
    if(currDis < minDis) {
      minDis = currDis;
      minIdx = i;
    }
  }

  ROS_INFO("minIdx: %d", minIdx);
  uController->setVref(4 * wps[minIdx][2]);

  std::vector<double> xs, ys, coeffs, currPos = uController->getCurrPos();
  double totalDist = 0, cosY = cos(uController->getCurrYaw()), sinY = sin(uController->getCurrYaw());
  while(minIdx != wps.size() && totalDist < lookaheadDist) {
    double localX = cosY * (wps[minIdx][0] - currPos[0]) - sinY * (wps[minIdx][1] - currPos[1]);
    double localY = sinY * (wps[minIdx][0] - currPos[0]) + cosY * (wps[minIdx][1] - currPos[1]);
    xs.push_back(wps[minIdx][0]);
    ys.push_back(wps[minIdx][1]);
    totalDist += wps_dist[minIdx];
    minIdx++;
  }

  coeffs = polyfit_boost(xs, ys, 3);
  std::reverse(coeffs.begin(), coeffs.end());
  ROS_INFO("coeffs: %.4f %.4f %.4f %.4f", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
  return coeffs;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "upolis_control");

  ros::NodeHandle n;

  string odomTopic, waypointsFilePath;
  n.getParam("/upolis_control/odom_topic", odomTopic);
  n.getParam("/upolis_control/waypoints_file", waypointsFilePath);


  // process waypoints csv file
  std::vector<std::vector<double> > waypointsVec, waypointsSorted;
  std::vector<double> wp_distance, w_X, w_Y;
  wp_distance.push_back(0.0);

  ifstream waypointsFile;
  waypointsFile.open(waypointsFilePath);
  string line;
  int lineno = 0;
  while (getline(waypointsFile, line)) {
      lineno++;
      stringstream ss(line);
      enum { NUM_ENTRIES = 3 };
      std:vector<double> v(NUM_ENTRIES);
      char delim;
      for (int i = 0; i < NUM_ENTRIES; i++) {
          if (!(ss >> v[i]))
              err_exit(lineno, i, "extract failed", line);
          else if (i < NUM_ENTRIES - 1 && !((ss >> delim) && delim == ','))
              err_exit(lineno, i, "delimiter incorrect", line);
          else if (i == NUM_ENTRIES - 1 && (ss >> delim))
              err_exit(lineno, i, "extra data at end of line", line);
      }
      if(lineno > 1) {
        double dist = std::sqrt(std::pow(v[0] - waypointsVec[lineno - 2][0] , 2) + std::pow(v[1] - waypointsVec[lineno - 2][1], 2));
        wp_distance.push_back(dist);
      }
      waypointsVec.push_back(v);
      w_X.push_back(v[0]);
      w_Y.push_back(v[1]);
  }
  waypointsFile.close();
  waypointsSorted = waypointsVec;
  std::sort(waypointsSorted.begin(), waypointsSorted.end());


  // Visualization
  string x_label = "X axis (m)",
        y_label = "Y axis (m)",
        axis_op = "equal",
        carMarker = "Car";
  
  plt::figure(1);
  plt::grid();
  plt::xlabel(x_label);
  plt::ylabel(y_label);
  plt::axis(axis_op);
  plt::title("Visuallizer");
  plt::ion();
  plt::show();
  
  // setup subs and pubs
  ros::Subscriber odomSub = n.subscribe(odomTopic, 10, odomCallback);
  ros::Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // initiate the controller
  uController = new uPolisControl(20, 0.1, 1.7, uPolisControl::MPC_WEIGHTS(), {-0.7, 0.7
  }, {-1, 1});
  
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(30);


  bool reachedEnd = false, stopVis = false;


  while (ros::ok())
  {
  
    ros::spinOnce();

    
    vector<double> currPos = uController->getCurrPos();
    currPos.push_back(INT_MIN); // for speed search
    if(calc_dist(currPos, waypointsVec[waypointsVec.size() - 1]) <= END_DIST_THRESHOLD) {
      ROS_INFO("Reached End!");
      reachedEnd = true;
    }

    if(!reachedEnd) {
      std::vector<double> traj = get_trajectory_coeffs(waypointsVec, wp_distance, currPos);
      currCoeff = &traj[0];
      uController->setCoffs(traj);
      uController->step();
      
    }



    std::pair<double, double> cmds = uController->getCommands();
    geometry_msgs::Twist cmdMsg;
    cmdMsg.linear.x = !reachedEnd ? cmds.first : 0;
    cmdMsg.angular.z = !reachedEnd ? cmds.second : 0;
    cmdVelPub.publish(cmdMsg);

    plt::clf();
    plt::plot(w_X, w_Y, {{"label", "Trajectory"}}); 
    plt::plot(std::vector<double>({currPos[0]}), std::vector<double>({currPos[1]}), {
      {"marker", "s"},
      {"color", "b"}
    });
    plt::text(currPos[0], currPos[1] + 0.3, carMarker);
    
    plt::draw();
    plt::pause(0.001);

    loop_rate.sleep();
  }
  uController->printResults();

  return 0;
}