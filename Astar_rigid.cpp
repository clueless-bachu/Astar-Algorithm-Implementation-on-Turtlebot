#include<bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include<cmath>
#define PI 3.14159265

struct VectorHash {
    size_t operator()(const std::vector<float>& v) const {
        std::hash<float> hasher;
        size_t seed = 0;
        for (float i : v) {
            seed ^= hasher(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }
        return seed;
    }
};


using vector3d = std::vector<std::vector<std::vector<float> > >;
using vector_set = std::unordered_set<std::vector<float>, VectorHash>;
using vector_map = std::unordered_map<std::vector<float>,std::vector<float>, VectorHash>;
using priority_q = std::priority_queue<std::tuple<float, std::vector<float>>,
 std::vector<std::tuple<float, std::vector<float>>>, std::greater<std::tuple<float, std::vector<float>>>>;
using tuple = std::tuple<float, std::vector<float>>;


cv::Scalar ScalarHSV2BGR(uchar H, uchar S, uchar V)
{
    cv::Mat rgb;
    cv::Mat hsv(1,1, CV_8UC3, cv::Scalar(H,S,V));
    cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
    return cv::Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

std::vector<float> img_to_cart(float i, float j)
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
    std::vector<float> cart_cords = {j, 200-i};
    return cart_cords;
}

std::vector<float> cart_to_img(float x, float y)
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

	std::vector<float> img_cords = {200-y, x};
    return img_cords;
} 

std::vector<float> get_straight(float x1,float y1,float x2, float y2, float r,float c,bool side=false,bool verbose=false)
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
    if(side) c1 = std::pow((r+c)*(1+m*m),0.5)+c_;
    else c1 = -std::pow((r+c)*(1+m*m),0.5)+c_;
    if(verbose) std::cout<<m<<" "<<c_<<" "<<c1<<std::endl;
    std::vector<float> params = {m,c1};
    return params;
}
   
bool indiamond(std::vector<float> state,float r=0,float c=0)
{
	/*
    Checks whether the given point is inside the diamond or not
    input:
    x: x-coordinate of given point
    y: y-coordinate of given point
    r: radius of the robot
    c: clearance of the robot
    returns:
    whether the point is inside the diamond or not
    */
    // cords = (225,10), (250,25), (225,40), (200,25)
    float x =state[0], y=state[1];

	std::vector<float> l1 = get_straight(225, 10, 250, 25, r, c);
    std::vector<float> l2 = get_straight(250, 25, 225, 40, r, c, true);
    std::vector<float> l3 = get_straight(225, 40, 200, 25, r, c, true);
    std::vector<float> l4 = get_straight(200, 25, 225, 10, r, c);
    
    return l1[0] * x + l1[1] <= y && y <= l2[0] * x + l2[1] && l3[0] * x + l3[1] >= y && y >= l4[0] * x + l4[1];
}
    
bool inconcave1(std::vector<float> state,float r=0,float c=0)
{
    /*
    Checks whether the given point is inside the concave polygon 1 or not
    input:
    x: x-coordinate of given point
    y: y-coordinate of given point
    r: radius of the robot
    c: clearance of the robot
    returns:
    whether the point is inside the concave polygon 1 or not
    */
	// cords: (25,185),(75,185),(50,150),(20,120)

	float x =state[0], y=state[1];

    std::vector<float> l1 = get_straight(25, 185, 75, 185, r, c, true);
    std::vector<float> l2 = get_straight(75, 185, 50, 150, 0, 0);
    std::vector<float> l3 = get_straight(50, 150, 20, 120, r, c);
    std::vector<float> l4 = get_straight(20, 120, 25, 185, r, c, true);
    
    return l1[0] * x + l1[1] >= y && y >= l2[0] * x + l2[1] && l3[0] * x + l3[1] <= y && y <= l4[0] * x + l4[1];
}

bool inconcave2(std::vector<float> state,float r=0,float c=0)
{
    /*
    Checks whether the given point is inside the concave polygon 2 or not
    input:
    x: x-coordinate of given point
    y: y-coordinate of given point
    r: radius of the robot
    c: clearance of the robot
    returns:
    whether the point is inside the concave polygon 2 or not
    */
	// cords: (75,185),(100,150),(75,120),(50,150)

	float x =state[0], y=state[1];
    std::vector<float> l1 = get_straight(75, 185, 100, 150, r, c, true);
    std::vector<float> l2 = get_straight(100, 150, 75, 120, r, c);
    std::vector<float> l3 = get_straight(75, 120, 50, 150, r, c);
    std::vector<float> l4 = get_straight(50, 150, 75, 185, 0, 0, true);
    
    return l1[0] * x + l1[1] >= y && y >= l2[0] * x + l2[1] && l3[0] * x + l3[1] <= y && y <=l4[0] * x + l4[1];
}

bool inrectangle(std::vector<float> state,float r=0,float c=0)
{
	/*
    Checks whether the given point is inside the rectangle or not
    input:
    x: x-coordinate of given point
    y: y-coordinate of given point
    r: radius of the robot
    c: clearance of the robot
    returns:
    whether the point is inside the rectangle or not
    */
	// cords: (95,30),(100,38.66),(35.05,76.16),(30.05,67.5)
    float x =state[0], y=state[1];
    std::vector<float> l1 = get_straight(95, 30, 100, 38.66, r, c);
    std::vector<float> l2 = get_straight(100, 38.66, 35.05, 76.16, r, c, true);
    std::vector<float> l3 = get_straight(35.05, 76.16, 30.05, 67.5, r, c, true);
    std::vector<float> l4 = get_straight(30.05, 67.5, 95, 30, r, c);

    return l1[0] * x + l1[1] <= y && y <= l2[0] * x + l2[1] && l3[0] * x + l3[1] >= y && y >= l4[0] * x + l4[1];
}

bool in_obstacle(std::vector<float> state, float r=0, float c=0)
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
	(x-225)*(x-225) + (y-150)*(y-150) <= (25+r+c)*(25+r+c) // for the circle
	|| (x-150)*(x-150)/((40+r+c)*(40+r+c))+(y-100)*(y-100)/((20+r+c)*(20+r+c)) <= 1 // for the ellipse
	|| indiamond(state,r,c) || inconcave1(state,r,c) || inconcave2(state,r,c) || inrectangle(state,r,c)
	|| x<0 || x>300 || y<0 || y>200;
	
}

std::vector<float> bin(std::vector<float> state, float scale1 = .5, float scale2= 6/PI)
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

std::vector<std::vector<float>> get_children(std::vector<float> state,float r=0, float c=0, 
    float dist = 1,float theta = PI/6)
{
    /*
    Explores the child nodes
    input:
    state: current coordinates
    r: radius of the robot
    c: clearance of the robot
    dist: stepsize  
    theta: angle between action set at each node
    returns:
    children: 2D vector of floats
    */
    
    float angles[] = {0,theta,2*theta,-theta, -2*theta};
    std::vector<std::vector<float>> children;

    for(int i=0; i< 5;++i)
    {
        std::vector<float> new_state(3);
        new_state[0] = state[0]+ dist*cos(state[2])*cos(angles[i])-dist*sin(state[2])*sin(angles[i]);
        new_state[1] = state[1]+ dist*sin(state[2])*cos(angles[i])+dist*cos(state[2])*sin(angles[i]);
        new_state[2] = state[2]+angles[i];
        if(new_state[2]<-PI) new_state[2] += 2*PI;
        else if(new_state[2]>PI) new_state[2] -=2*PI; 

        if(!in_obstacle(new_state,r,c))
        {
            children.push_back(new_state);
        }
    }
    return children;

}

float euclidean_dist(std::vector<float> pos, std::vector<float> goal)
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


std::tuple<vector_map,  vector3d, std::vector<float> > a_star(std::vector<float> start, std::vector<float> goal,
 float r=0, float c=0, float dist = 1,  float theta = PI/6, float thresh =1.5)
{   
   /*
   Explores the child nodes
   input:
   start: start coordinates
   goal: coordinates to reach
   r: radius of the robot
   c: clearance of the robot
   dist: stepsize distance
   theta: angle between action set at each node 
   thresh: threshold distance around the goal
   returns:
   a tuple 'path' which consists of 'bactrack' (which maps node to its parent node), 'memory' (list of all the nodes explored) 
   and 'cur_state'(the last node that was explored)
   */

	vector_set visited;
	std::vector<std::vector<float>> children;
	vector3d memory;
	vector_map backtrack;
	priority_q q;
	float cost;
	std::vector<float> cur_state;
	tuple begin(0,start);
	q.push(begin);

	if(in_obstacle(goal) || in_obstacle(start))
	{
		std::cout<<"Check your inputs"<<std::endl;
		std::vector<float> tmp;
		std::tuple<vector_map,  vector3d, std::vector<float> > path(backtrack, memory,tmp);
		return path;
	}
	unsigned int count =0 ;
	while(!q.empty())
	{
  		std::tie (cost, cur_state) = q.top();
  		q.pop();
  		
  		if(euclidean_dist(cur_state,goal)<=thresh) 
  		{
  			std::cout<<"Goal Reached!!!"<<std::endl;\

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
  			backtrack.insert(std::pair<std::vector<float>,std::vector<float>>(child,cur_state));
  		}
  		children.push_back(cur_state);
  		memory.push_back(children);
  		++count;

	}
	std::cout<<"Final State: ["<<cur_state[0]<<"\t\t"<<cur_state[1]<<"\t\t"<<cur_state[2]*180/PI<<"]\n"<<"Number of Nodes Explored: "
	<<count<<std::endl;
	std::tuple<vector_map,  vector3d, std::vector<float> > path(backtrack, memory, cur_state);
	return path;
}

int main()
{
//////////////////////////INITIALIZATION&INPUTS///////////////////////////////////////////////////////////////////////////
	float r,c, thresh,dist,ang;
	int scale=4;// For resizing
	float x1,x2,y1,y2;
	std::vector<float> parent_img,parent_cart,child_img, child_cart;
	cv::Point_<float> parent,child;
	int speed = 200;
	std::vector<float> start(3), goal(3), last_node(3);
	vector_map backtrack;
    vector3d memory;

	std::cout<<"Please input radius and clearance as \"r c\" (without the quotes)\n";
	std::cin>>r>>c;
	std::cout<<
	"Please input starting configuration as \"x y theta\" (without the quotes) where theta is in degrees\n";
	std::cin>>start[0]>>start[1]>>start[2];
	std::cout<<
	"Please input goal configuration as \"x y theta\" (without the quotes) where theta is in degrees\n";
	std::cin>>goal[0]>>goal[1]>>goal[2];
	std::cout<<"Please input the threshold for reaching near the goal\n";
	std::cin>>thresh;
	std::cout<<"Please input the step size and angle of turn in degrees for reaching near the goal\n";
	std::cin>>dist>>ang;
	
	ang = ang*PI/180;
	start[2] = start[2]*PI/180;


//////////////////A* ALGORITHM////////////////////////////////////////////////////////////////////////////////////////////////
	
    auto start_time = std::chrono::high_resolution_clock::now();
    std::tie(backtrack, memory, last_node) = a_star(start, goal,r,c,dist,ang,thresh);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start_time); 


    std::cout << "Time taken by function: "
         << duration.count() << " seconds" << std::endl;

///////////////////VISUALISATION/////////////////////////////////////////////////////////////////////////////////////////////////////////////
   	cv::Mat resized, img(200, 300, CV_8UC3, cv::Scalar(200,200, 200));
   	cv::namedWindow("A* in Action", cv::WINDOW_AUTOSIZE);

   	for(int i = 0; i < img.rows; i++)
	{
	    for(int j = 0; j < img.cols; j++)
	    {
	    	std::vector<float> cords = img_to_cart(i,j);
	    	std::vector<float> state = {cords[0],cords[1],0};

	    	if(in_obstacle(state,r,c))
	    	{
	    		 cv::Vec3b pixel = {0,0,0};
	    		 img.at<cv::Vec3b>(i, j)= pixel;
	    	}
	    }
	}

	
	cv::resize(img, resized,cv::Size(), scale,scale);
	// cv::VideoWriter video("output.avi",cv::VideoWriter::fourcc('M','J','P','G'),120, cv::Size(300*scale, 200*scale));

	
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
			cv::line(resized, parent, child,ScalarHSV2BGR((node/speed)%255,255, 255),1);
			if(!(node%speed))
			{
				// video.write(resized);
				cv::imshow("A* in Action", resized);
				cv::waitKey(1);
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
		cv::line(resized, parent, child,cv::Scalar(255,255, 255),2);
		// video.write(resized);
		cv::imshow("A* in Action", resized);
		cv::waitKey(1);
		goal = child_cart;
	}
	// video.release();
   	cv::imshow("A* in Action", resized);
   	cv::waitKey(0); 	
    
    return 0;
}
