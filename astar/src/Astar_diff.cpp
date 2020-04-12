#include<bits/stdc++.h>
#include<unistd.h>
// #include "opencv2/opencv.hpp"
#include<cmath>
#include <tf/transform_broadcaster.h>
#define PI 3.14159265
#include "ros/ros.h"
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;
// using namespace cv;



struct VectorHash {
    size_t operator()(const vector<float>& v) const {
        hash<float> hasher;
        size_t seed = 0;
        for (float i : v) {
            seed ^= hasher(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }
        return seed;
    }
};


struct cost_comparator
{
    bool operator()(tuple<float, vector<float>> const& a, tuple<float, vector<float>> const& b) const
    {

        return  get<1>(a)>get<1>(b);
    }
};
using vector3d = vector<vector<vector<float> > >;
using vector_set = unordered_set<vector<float>, VectorHash>;
using vector_map = unordered_map<vector<float>,vector<float>, VectorHash>;
using priority_q = priority_queue<tuple<float, vector<float>>,
 vector<tuple<float, vector<float>>>, greater<tuple<float, vector<float>>>>;
using tple = tuple<float, vector<float>>;

	
// Scalar ScalarHSV2BGR(uchar H, uchar S, uchar V)
// {
//     Mat rgb;
//     Mat hsv(1,1, CV_8UC3, Scalar(H,S,V));
//     cvtColor(hsv, rgb, COLOR_HSV2BGR);
//     return Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
// }

vector<float> img_to_cart(float i, float j)
{
	/*
    Converts an image coordinates to cartesian coordinates
    input:
    i: row of image
    j: column of image
    returns:
    x: x-coordinate in cartesian system
    y: y-coordinate in cartesian system
    */
    vector<float> cart_cords = {(j-250)/50, (250-i)/50};
    return cart_cords;
}

vector<float> cart_to_img(float x, float y)
{
    /*
    Converts cartesian coordinates to image coordinates
    input:
    x: x-coordinate in cartesian system
    y: y-coordinate in cartesian system
    returns:
    i: row of image
    j: column of image
    */

	vector<float> img_cords = {250-50*y, 50*x+250};
    return img_cords;
} 

bool in_square1(vector<float> state,float r=0,float c=0)
{
    //0.25, 1.75->x, 4.25,5.75-> y
    float x = state[0], y = state[1];
    return  x>=-4.75-r-c && x<=-3.25+r+c && y>= -0.75-r-c && y<= 0.75+r+c;
}

bool in_square2(vector<float> state,float r=0,float c=0)
{
    //2.25, 3.75->x, 7.45,8.95-> y
    float x = state[0], y = state[1];
    return x>=-2.75-r-c && x<=-1.25+r+c && y>= 2.25-r-c && y<= 3.75+r+c;
}

bool in_square3(vector<float> state,float r=0,float c=0)
{
    //0.25, 1.75->x, 4.25,5.75-> y
    float x = state[0], y = state[1];
    return x>=3.25-r-c && x<=4.75+r+c && y>= -0.75-r-c && y<= 0.75+r+c;
}

bool in_obstacle(const vector<float>& state, float r=0, float c=0)
{
    /*
    Checks whether the state is inside the obstacle space
    input:
    state: current coordinates
    m_map: modified map considering radius and clearance (??)
    returns:
    a boolean whether the state is in obstacle space or not
    */
    float x =state[0], y=state[1];  

    return 
    (x)*(x) + (y)*(y) <= (1+r+c)*(1+r+c) || 
    (x-2)*(x-2) + (y-3)*(y-3) <= (1+r+c)*(1+r+c) ||
    (x-2)*(x-2) + (y+3)*(y+3) <= (1+r+c)*(1+r+c) ||
    (x+2)*(x+2) + (y+3)*(y+3) <= (1+r+c)*(1+r+c) ||
    in_square1(state,r,c)||in_square2(state,r,c)||in_square3(state,r,c)||
    x<-5-r-c || x>5+r+c||y<-5-r-c||y>5+r+c;
}



vector<float> bin(vector<float> state, float scale1 = 0.05, float scale2= 8/PI)
{
    /*
    computes the bin a state belongs to in the configuration space
    input:
    state: current coordinates
    
    returns:
    the bin state
    */
    
    state[0] = ((int)((1.0/scale1)*state[0]))*scale1;
    state[1] = ((int)((1.0/scale1)*state[1]))*scale1;
    state[2] = ((int)((1.0/scale2)*state[2]))*scale2;
    
    return state;
}

 
vector<vector<float>> get_children(vector<float> state, float rpm1, float rpm2,const float &r,const float &c ,
	const float &wr ,const float &L, float dt = 1)
{
    /*
    Explores the child nodes
    input:
    state: current coordinates (x,y,theta)
    rpm1: first input rpm from user
    rpm2: second input rpm from user
    r: wheel radius of the robot
    L: distance between 2 wheels
    c: clearance of the robot
    dt: differential stepsize  
    returns:
    children: 2D vector of floats
    */
    
    rpm1 = rpm1*(PI/30); // changed to radian per sec
    rpm2 = rpm2*(PI/30);  // changed to radian per sec
	
    float velocity[8][2] = {{0,rpm1},{rpm1,0},{rpm1,rpm1},{0,rpm2},{rpm2,0},{rpm2,rpm2},{rpm1,rpm2},{rpm2,rpm1}}; // 8 action spaces
    vector<vector<float>> children;
	float ul, ur,omega, i_radius, iccx, iccy;
    std::vector<float> new_state(5);
	
    for(int i=0; i< 8;++i)
    {
    	ul = velocity[i][0]*wr;
    	ur = velocity[i][1]*wr;
        // printf("%f %f\n",ul,ur);    
       
        if(ul==ur)
        {
            new_state[0] = state[0]+ ul*dt*cos(state[2]);
            new_state[1] = state[1]+ ul*dt*sin(state[2]);
            new_state[2] = state[2];
        }
        else
        {
            omega = (ur-ul)/L ;
	        i_radius = (ur+ul)/(2*omega);

	        iccx = state[0]- i_radius*sin(state[2]);
	        iccy = state[1]+ i_radius*cos(state[2]);

	        new_state[0] = (state[0]-iccx)*cos(omega*dt)-(state[1]-iccy)*sin(omega*dt)+iccx;//state[0]+(0.5*wr)*(ul+ur)*cos(state[2])*dt;                                
	        new_state[1] = (state[0]-iccx)*sin(omega*dt)+(state[1]-iccy)*cos(omega*dt)+iccy;//state[1]+(0.5*wr)*(ul+ur)*sin(state[2])*dt;                                
	        new_state[2] = omega*dt+state[2];//state[2]+(wr/L)*(ur-ul)*dt;   
        }

                                                    
        while(new_state[2]<-PI) new_state[2] += 2*PI;
        while(new_state[2]>PI) new_state[2] -=2*PI; 

        new_state[3] = ul/wr;
        new_state[4] = ur/wr;
        
        if(!in_obstacle(new_state,r,c))
        {
            children.push_back(new_state);
        }
    }
    return children;
}

float euclidean_dist(vector<float> pos, vector<float> goal)
{
    /*
    Calculates the euclidean distance rom the goal 
    input:
    pos: the current postion of the robot
    goal: the destination t reach
    returns:
    h: the euclidean distance between goal and current position of robot
    */
    return pow(pow((goal[0]-pos[0]),2) +pow((goal[1]-pos[1]),2),0.5);
    
}


tuple<vector_map,  vector3d, vector<float> > a_star(vector<float> start, vector<float> goal,
const float &r,const float &c, const float &wr, const float &l, float rpm_1,  float rpm_2, const float &thresh)
{   
   /*
   Explores the child nodes
   input:
   start: current coordinates
   goal: coordinates to reach
   r: radius of the robot
   c: clearance of the robot
   dist: distance (??) 
   theta: angle between action set at each node      
   returns:
   */

    vector_set visited;
    vector<vector<float>> children;
    vector3d memory;
    vector_map backtrack;
    priority_q q;
    float cost;
    vector<float> cur_state;
    tple begin(0,start);
    q.push(begin);

    if(in_obstacle(goal) || in_obstacle(start))
    {
        cout<<"Check your inputs"<<endl;
        vector<float> tmp;
        tuple<vector_map,  vector3d, vector<float> > path(backtrack, memory,tmp);
        return path;
    }
    unsigned int count =0 ;
    while(!q.empty())
    {

        tie (cost, cur_state) = q.top();
        q.pop();
        

        if(euclidean_dist(cur_state,goal)<=thresh) 
        {
            cout<<"Goal Reached!!!"<<endl;\

            break;
        }
        else if(visited.find(bin(cur_state))!=visited.end()) continue;
        visited.insert(bin(cur_state));
        //(vector<float> state, float rpm1, float rpm2, float r, float c , float L, float dt = 1)
        children = get_children(cur_state,rpm_1, rpm_2,r,c,wr,l);
        
        for(auto child: children)
        {
            float child_cost = cost+1+euclidean_dist(child,goal);
            tple node(child_cost, child);
            q.push(node);
            backtrack.insert(pair<vector<float>,vector<float>>(child,cur_state));
            // printf("child: %f\t%f\t%f\n",child[0],child[1],child[2]);
        }

        children.push_back(cur_state);
        memory.push_back(children);
        ++count;

    }
    cout<<"Final State: ["<<cur_state[0]<<"\t\t"<<cur_state[1]<<"\t\t"<<cur_state[2]*180/PI<<"]\n"<<"Number of Nodes Explored: "
    <<count<<endl;
    tuple<vector_map,  vector3d, vector<float> > path(backtrack, memory, cur_state);

    if(euclidean_dist(cur_state,goal)>thresh)
    {
        cout<<"Path Not Found";
        exit(1);
    }
    return path;
}


class actuator
{ 
private:
	ros::Publisher pub;
	ros::NodeHandle n;

public:
	actuator(void)
	{

		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
		// sub = n.subscribe("/gps", 1, &actuator::callback,this);
	}

	void actuatorcb(float rpml,float rpmr,float wr,float L)
	{
		// wr = 0.33;
		// L = 0.9;
		float ur, ul, v, omega, i_radius;
		ur = rpmr*wr;
		ul = rpml*wr;
		if (rpml == rpmr)
		{
	 		v = ur;
			omega = 0;
		}		
		else 
		{		
			omega = (ur-ul)/L ;
	       	i_radius = (ur+ul)/(2*omega);
			v = omega*i_radius;
		 }
		geometry_msgs::Twist msg;
		msg.linear.x = v;
		// msg.linear.y = -0.1;

		msg.angular.z = omega;
		// printf("%f %f\n",msg.linear.x,msg.angular.z);

	    pub.publish(msg);


	}

};

int main(int argc,char *argv[])
{

	
//////////////////////////INITIALIZATION&INPUTS///////////////////////////////////////////////////////////////////////////
    float wr,l,r,c, thresh,rpm_1, rpm_2;
    ros::init(argc,argv,"actuator");
	actuator bot;
    float x1,x2,y1,y2;
    vector<float> parent_img,parent_cart,child_img, child_cart;
    int speed;
    vector<float> start(5), goal(5), last_node(5);
    vector_map backtrack;
    vector3d memory;



    start = {atof(argv[1]),atof(argv[2]), atof(argv[3]),0,0};
    goal = {atof(argv[4]),atof(argv[5]),0,0,0};
    r = 0.105; 
    //c = 0.4;
    wr = 0.033; l = 0.16;
    c = atof(argv[6]);
    // cout<<c<<endl;
    rpm_1 = atof(argv[7]); rpm_2 = atof(argv[8]); 
    thresh =0.1;
    start[2] = start[2]*PI/180;
    sleep(10);
    cout<<"----------------------------Starting Planning------------------------\n";
// // //////////////////A* ALGORITHM////////////////////////////////////////////////////////////////////////////////////////////////
    
    auto start_time = chrono::high_resolution_clock::now();
    tie(backtrack, memory, last_node) = a_star(start, goal,r,c,wr,l,rpm_1,rpm_2,thresh);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::seconds>(stop - start_time); 


    cout << "Time taken by function: "
         << duration.count() << " seconds" << endl;

// ///////////////////VISUALISATION/////////////////////////////////////////////////////////////////////////////////////////////////////////////



    goal = last_node;
    vector<vector<float>> actions;

    while(goal!= start)
    {
        child_cart = backtrack[goal];
        actions.push_back(child_cart);
        goal = child_cart;
    }
    

    for(int i=actions.size()-1; i>=0; --i)
    {
        bot.actuatorcb(actions[i][3],actions[i][4],wr,l);
		sleep(1.05);
    }
    cout<<"END\n"<<endl;
    for(int i=0; i<10; ++i)
    bot.actuatorcb(0,0,wr,l);       
    
    return 0;
}