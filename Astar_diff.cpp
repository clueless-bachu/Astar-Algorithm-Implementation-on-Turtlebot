#include<bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include<cmath>
#define PI 3.14159265

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
using tuple = tuple<float, vector<float>>;


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
    vector<float> cart_cords = {j, 200-i};
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

	vector<float> img_cords = {200-y, x};
    return img_cords;
} 

vector<float> get_straight(float x1,float y1,float x2, float y2, float r,float c,bool side=false,bool verbose=false)
{
	/*
    Creates a straight line using two points in the cartesian system
    input:
    x1: x-coordinate of point 1 in cartesian system
    y1: y-coordinate of point 1 in cartesian system
    x2: x-coordinate of point 2 in cartesian system
    y2: y-coordinate of point 2 in cartesian system
    r: radius of the robot
    c: clearance of the robot
    returns:
    m: slope of the  of the straight line
    c1: y intercept of the straight line
    */
    float m = (y2-y1)/(x2-x1);
    float c_ = y1 - x1*(y2-y1)/(x2-x1);
    float c1;
    if(side) c1 = pow((r+c)*(1+m*m),0.5)+c_;
    else c1 = -pow((r+c)*(1+m*m),0.5)+c_;
    if(verbose) cout<<m<<" "<<c_<<" "<<c1<<endl;
    vector<float> params = {m,c1};
    return params;
}

// bool insquare1,2,3, inobstacle
bool in_square1(vector<float> state,float r=0,float c=0)
{
    //0.25, 1.75->x, 4.25,5.75-> y
    float x = state[0], y = state[1];
    return  x>=-5.75-r-c && x<=-4.25+r+c && y>= -0.75-r-c && y<= 0.75+r+c;
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
    return x>=4.25-r-c && x<=5.75+r+c && y>= -0.75-r-c && y<= 0.75+r+c;
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
    (x)*(x) + (y)*(y) <= (2+r+c)*(2+r+c) || 
    (x-2)*(x-2) + (y-3)*(y-3) <= (2+r+c)*(2+r+c) ||
    (x-2)*(x-2) + (y+3)*(y+3) <= (2+r+c)*(2+r+c) ||
    (x+2)*(x+2) + (y+3)*(y+3) <= (2+r+c)*(2+r+c) ||
    in_square1(state,r,c)||in_square2(state,r,c)||in_square3(state,r,c)||
    x<-5-r-c || x>5+r+c||y<-5-r-c||y>5+r+c;
}



vector<float> bin(vector<float> state, float scale1 = 0.5, float scale2= 8/PI)
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

  std::vector<std::vector<float>> get_children(std::vector<float> state, std::vector<float> rpm, float r , float L, float c=0, 
    float dist = 1,float theta = PI/8)
{
    /*
    Explores the child nodes
    input:
    state: current coordinates (x,y,theta)
    rpm: speed of left and right wheels
    r: wheel radius of the robot
    L: distance between 2 wheels
    c: clearance of the robot
    dist: stepsize  
    theta: angle between action set at each node
    returns:
    children: 2D vector of floats
    */
    
    float angles[] = {0, theta, 2*theta, 3*theta, 4*theta, -theta, -2*theta, -3*theta}; // 8 action spaces
    std::vector<std::vector<float>> children;
	dt = dist;
	ul = rpm[0];
	ur = rpm[1];
		
    for(int i=0; i< 8;++i)
    {
        std::vector<float> new_state(3);
        new_state[0] = (0.5*r)*(ul+ur)*cos(state[2])*dt;                                
        new_state[1] = (0.5*r)*(ul+ur)*cos(state[2])*dt;                                
        new_state[2] = (r/L)*(ur-ul)*dt;                                                
        if(new_state[2]<-PI) new_state[2] += 2*PI;
        else if(new_state[2]>PI) new_state[2] -=2*PI; 

        if(!in_obstacle(new_state,rpm,c))
        {
            children.push_back(new_state);
        }
    }
    return children;


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
 float r=0, float c=0, float dist = 1,  float theta = PI/6, float thresh =1.5)
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
    tuple begin(0,start);
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
        children = get_children(cur_state,r,c, dist, theta);
        
        for(auto child: children)
        {
            float child_cost = cost+1+euclidean_dist(child,goal);
            tuple node(child_cost, child);
            q.push(node);
            backtrack.insert(pair<vector<float>,vector<float>>(child,cur_state));
        }
        children.push_back(cur_state);
        memory.push_back(children);
        ++count;

    }
    cout<<"Final State: ["<<cur_state[0]<<"\t\t"<<cur_state[1]<<"\t\t"<<cur_state[2]*180/PI<<"]\n"<<"Number of Nodes Explored: "
    <<count<<endl;
    tuple<vector_map,  vector3d, vector<float> > path(backtrack, memory, cur_state);
    return path;
}

int main()
{
//////////////////////////INITIALIZATION&INPUTS///////////////////////////////////////////////////////////////////////////
    float r,c, thresh,dist,ang;
    int scale=4;// For resizing
    float x1,x2,y1,y2;
    vector<float> parent_img,parent_cart,child_img, child_cart;
    Point_<float> parent,child;
    int speed = 200;
    vector<float> start(3), goal(3), last_node(3);
    vector_map backtrack;
    vector3d memory;

    cout<<"Please input radius and clearance as \"r c\" (without the quotes)\n";
    cin>>r>>c;
    cout<<
    "Please input starting configuration as \"x y theta\" (without the quotes) where theta is in degrees\n";
    cin>>start[0]>>start[1]>>start[2];
    cout<<
    "Please input goal configuration as \"x y theta\" (without the quotes) where theta is in degrees\n";
    cin>>goal[0]>>goal[1]>>goal[2];
    cout<<"Please input the threshold for reaching near the goal\n";
    cin>>thresh;
    cout<<"Please input the step size and angle of turn in degrees for reaching near the goal\n";
    cin>>dist>>ang;
    
    ang = ang*PI/180;
    start[2] = start[2]*PI/180;


//////////////////A* ALGORITHM////////////////////////////////////////////////////////////////////////////////////////////////
    
    auto start_time = chrono::high_resolution_clock::now();
    tie(backtrack, memory, last_node) = a_star(start, goal,r,c,dist,ang,thresh);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::seconds>(stop - start_time); 


    cout << "Time taken by function: "
         << duration.count() << " seconds" << endl;

///////////////////VISUALISATION/////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Mat resized, img(200, 300, CV_8UC3, Scalar(200,200, 200));
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

    
    resize(img, resized,Size(), scale,scale);
    // VideoWriter video("output.avi",VideoWriter::fourcc('M','J','P','G'),120, Size(300*scale, 200*scale));

    
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
            line(resized, parent, child,ScalarHSV2BGR((node/speed)%255,255, 255),1);
            if(!(node%speed))
            {
                // video.write(resized);
                imshow("A* in Action", resized);
                waitKey(1);
            }
        }
    }

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
        line(resized, parent, child,Scalar(255,255, 255),2);
        // video.write(resized);
        imshow("A* in Action", resized);
        waitKey(1);
        goal = child_cart;
    }
    // video.release();
    imshow("A* in Action", resized);
    waitKey(0);     
    
    return 0;
}
