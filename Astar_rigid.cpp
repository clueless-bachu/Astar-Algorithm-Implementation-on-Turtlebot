#include<bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include<cmath>
#define PI 3.14159265
#include<unordered_set>



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
    std::vector<float> cart_cords = {j, 199-i};
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

	std::vector<float> img_cords = {199-y, x};
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
	|| indiamond(state,r,c) || inconcave1(state,r,c) || inconcave2(state,r,c) || inrectangle(state,r,c);
	
}

std::vector<float> bin(std::vector<float> state, float scale1 = 1.0, float scale2= 0.02)
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
    float dist = 3,float theta = PI/6)
{
    /*
    Explores the child nodes
    input:
    state: current coordinates
    r: radius of the robot
    c: clearance of the robot
    dist: distance (??) 
    theta: angle between action set at each node
    returns:
    children: dictionary which maps the child coordinates to cost (??)
    */
    
    float angles[] = {0,theta,2*theta,-theta, -2*theta};
    std::vector<std::vector<float>> children;

    for(int i=0; i< 5;++i)
    {
        std::vector<float> new_state(3);
        new_state[0] = state[0]+ dist*cos(state[2])*cos(angles[i])-dist*sin(state[2])*sin(angles[i]);
        new_state[1] = state[0]+ dist*sin(state[2])*cos(angles[i])+dist*cos(state[2])*sin(angles[i]);
        new_state[2] = state[2]+angles[i];
        if(!in_obstacle(new_state,r,c))
        {
            children.push_back(new_state);
        }
    }
    return children;

}

float heuristic(std::vector<float> pos, std::vector<float> goal)
{
	/*
    Calculates the eucledian distance rom the goal 
	input:
	pos: the current postion of the robot
	goal: the destination t reach
	returns:
	h: the eucledian distance between goal and current position of robot
	*/
	float h = sqrt((goal[0]-pos[0])^2 +(goal[1]-pos[1])^2);
	return h;
	
}

std::vector<std::vector<float>> a_star(std::vector<float> start, std::vector<float> goal, float r=0, float c=0, float dist = 3,  float theta = PI/6)
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
   std::vector<float> memory;
   std::vector<float> backtrack;
   queue <float> nodes[]={start};
    
   std::vector<float> costs(3);
   unordered_set<std::vector<float>>state_cost;
   std::vector<float> costs(3);
   costs[0] = std::numeric_limits<double>::infinity() ; // cost to come
   costs[1] =  heuristic(cord, goal); //hueristic cost
   costs[2] = costs[0]+costs[1]; //total cost
   state_cost.insert(make_pair(cord, costs))
   
   unordered_set <std::vector<float>>visited;
     
   while( !(nodes.empty()))
   {
   	make_pair(float cost, std::vector<float> cord) = nodes.pop();
   	if (cord == goal)
   	{
   		break;
	   }
	else
	{
		visited.insert(cord);
		memory.push_back(cord);
		std::vector<std::vector<float>> child_cord = get_children(cord, r, c, dist, theta)[0];
		std::vector<std::vector<float>> child_cost = get_children(cord, r, c, dist, theta)[1];
		for (i=0; i<child_cord.size(); ++i)
		{
		 if (visited.find(child_cord) != visited.end())	
		 {
		 	if (state_cost(child_cord, costs[2]) >= child_cost+ heurisitic(child_cord,goal))
		 	{
		 	 state_cost(child_cord, costs[2]) = child_cost+ heurisitic(child_cord,goal);
		 	 backtrack.push_back(child_cord);
			 }
			nodes.push(child_cord);
		 }
		}
	   }
	}
	return backtrack;
}



int main()
{
	// float r,c,xs,ys,thetas,xg,yg,thetag;
	// std::cout<<"Please input radius and clearance as \"r c\" (without the quotes)\n";
	// std::cin>>r>>c;
	// std::cout<<
	// "Please input starting configuration as \"x y theta\" (without the quotes) where theta is in radians\n";
	// std::cin>>xs>>ys>>thetas;
	// std::cout<<
	// "Please input goal configuration as \"x y theta\" (without the quotes) where theta is in radians\n";
	// std::cin>>xg>>yg>>thetag;

    std::vector<float> v = {10,10,PI};

    std::vector<std::vector<float> > children = get_children(v);

    std::cout<<children.size()<<std::endl;
    for(auto i: children)
    {
        for(auto j: i) std::cout<<j<<" ";
        std::cout<<std::endl;
    }

    return 0;
}
