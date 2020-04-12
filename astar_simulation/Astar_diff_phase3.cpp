#include<bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include<cmath>
// #include <tf/transform_broadcaster.h>
#define PI 3.14159265
// #include "ros/ros.h"
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;



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

	
Scalar ScalarHSV2BGR(uchar H, uchar S, uchar V)
{
    Mat rgb;
    Mat hsv(1,1, CV_8UC3, Scalar(H,S,V));
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
    return Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

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

bool in_obstacle(vector<float> state, float r=0, float c=0)
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
    std::vector<float> new_state(3);
	
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
        // printf("angle:%f\n", new_state[2]);
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
        exit(0);
        // vector<float> tmp;
        // tuple<vector_map,  vector3d, vector<float> > path(backtrack, memory,tmp);
        // return path;
    }
    unsigned int count =0 ;
    while(!q.empty())
    {

        tie (cost, cur_state) = q.top();
        q.pop();
        // printf("cur_state: %f\t%f\t%f\n",cur_state[0],cur_state[1],cur_state[2]);

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


// class actuator
// { 
// private:
// 	ros::Publisher pub;
// 	// ros::Subscriber sub;
// 	ros::NodeHandle n;
// 	// control::Actuator angles;
// 	// bool flag;
// 	// float par, gait[12];
// 	// std::queue<Eigen::Vector3f> pos;

// public:
// 	actuator(void)
// 	{
// 		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
// 		// sub = n.subscribe("/gps", 1, &actuator::callback,this);
// 	}

// 	void actuatorcb(void)
// 	{
// 		//pub.publish();
// 		// ros::spinOnce();
// 		geometry_msgs::Twist msg;
// 		msg.linear.x = -0.1;
// 		// msg.linear.y = -0.1;

// 		msg.angular.z = 1;

// 	    //publish the message
// 	    pub.publish(msg);


// 	}

// };

int main(int argc,char **argv)
{

	// ros::init(argc,argv,"actuator");
	// actuator bot;
//////////////////////////INITIALIZATION&INPUTS///////////////////////////////////////////////////////////////////////////
    float wr,l,r,c, thresh,rpm_1, rpm_2;
    int scale=1;
    float x1,x2,y1,y2;
    vector<float> parent_img,parent_cart,child_img, child_cart;
    Point_<float> parent,child;
    int speed;
    vector<float> start(3), goal(3), last_node(3);
    vector_map backtrack;
    vector3d memory;

    r = 0.105; c = 0.2; wr = 0.033; l = 0.16;
     // while (ros::ok())
     // {
     // 	bot.actuatorcb();
     // 	ros::spinOnce();
     // }
    // cout<<"sc;
    cout<<"Please input the clearance\n";
    cin>>c;
    cout<<
    "Please input starting configuration as \"x y theta\" (without the quotes) where theta is in degrees\n";
    cin>>start[0]>>start[1]>>start[2];
    cout<<
    "Please input goal configuration as \"x y theta\" (without the quotes) where theta is in degrees\n";
    cin>>goal[0]>>goal[1]>>goal[2];
    // cout<<"Please input the threshold for reaching near the goal\n";
    // cin>>thresh;
    cout<<"Please input the rpm1 and rpm2\n";
    cin>>rpm_1>>rpm_2;
    thresh = 0.1;
    start[2] = start[2]*PI/180;


// //////////////////A* ALGORITHM////////////////////////////////////////////////////////////////////////////////////////////////
    
    auto start_time = chrono::high_resolution_clock::now();
    tie(backtrack, memory, last_node) = a_star(start, goal,r,c,wr,l,rpm_1,rpm_2,thresh);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::seconds>(stop - start_time); 


    cout << "Time taken by function: "
         << duration.count() << " seconds" << endl;

// ///////////////////VISUALISATION/////////////////////////////////////////////////////////////////////////////////////////////////////////////

        Mat resized, img(500, 500, CV_8UC3, Scalar(255,255, 255));
        namedWindow("A* in Action", WINDOW_AUTOSIZE);

        for(int i = 0; i < img.rows; i++)
        {
            for(int j = 0; j < img.cols; j++)
            {
                vector<float> cords = img_to_cart(i,j);
                vector<float> state = {cords[0],cords[1],0};

                if(in_obstacle(state,r,c))
                {
                     Vec3b pixel = {0,0,0};
                     img.at<Vec3b>(i, j)= pixel;
                }
            }
        }
        // imshow("A* in Action", img);
        // waitKey(0);
        speed = memory.size()/100+1;

        
        resize(img, resized,Size(), scale,scale);
        // VideoWriter video("phase_3.avi",VideoWriter::fourcc('M','J','P','G'),50, Size(500*scale, 500*scale));
        cout<<backtrack.size()<<endl;
        
        for(auto node = 0; node< memory.size(); ++node)
        {
            x1 =  memory[node][memory[node].size()-1][0];
            y1 =  memory[node][memory[node].size()-1][1];
            // parent_cart = {x1,y1};
            parent_img  = cart_to_img(x1,y1);
            parent.x = parent_img[1]*scale;
            parent.y = parent_img[0]*scale; 

            for(int i=0;i< memory[node].size()-1;++i)
            {
                x2 =  memory[node][i][0];
                y2 =  memory[node][i][1];
                child_cart = {x1,y1};
                child_img  = cart_to_img(x2,y2);
                child.x = child_img[1]*scale;
                child.y = child_img[0]*scale;
                // printf("%f %f %f %f\n",child.x, child.y, parent.x, parent.y);
                line(resized, parent, child,ScalarHSV2BGR((node/speed)%255,255, 255),1);
                if(!(node%speed))
                {
                    // video.write(resized);
                    imshow("A* in Action", resized);
                    waitKey(10);
                }
            }
        }
        int count=0;
        goal = last_node;
        while(goal!= start)
        {
            child_cart = backtrack[goal];
            parent_cart = goal;
            parent_img  = cart_to_img(parent_cart[0],parent_cart[1]);
            child_img  = cart_to_img(child_cart[0],child_cart[1]);
            parent.x = parent_img[1]*scale;
            parent.y = parent_img[0]*scale;
            child.x = child_img[1]*scale;
            child.y = child_img[0]*scale;
            line(resized, parent, child,Scalar(0,0,0),2);
            ++count;
            // video.write(resized);
            imshow("A* in Action", resized);
            waitKey(1);
            goal = child_cart;
        }
        // video.release();
        imshow("A* in Action", resized);
        waitKey(1);     
        cout<<count<<endl;
    return 0;
}
