#include<bits/stdc++.h>
#include "opencv2/opencv.hpp"



// get_children(state, actions, action_costs, m_map, stepsize, theta)
// {
// 	  Explores the child nodes
//     input:
//     state: current coordinates
//     actions: specified set od actions possible
//     action_costs: costs of actions
//     m_map: modified map considering radius and clearance (??)
//     stepsize: stepsize of movement of radius
//     theta: angle between action set at each node
//     returns:
//     children: dictionary which maps the child coordinates to cost (??)
//     total_cost: total cost of the child node (??)
    
    
//     // code
// }

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

bool state_validity(std::vector<float> state, float r=0, float c=0)
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

// m_map(map, radius, clearance)
// {
// 	/*
// 	Gives modified obstacle space
// 	input:
// 	map: original obstacle space
// 	radius: radius of rigid robot
// 	clearance: clearance of rigid robot
// 	returns:
// 	m_map: modified map considering radius and clearance (??)
// 	*/
	
// 	// code	
// }

// A_star(start, goal, threshold, m_map, actions, action_costs, heuristic)
// {
// 	  Explores the child nodes
//     input:
//     start: current coordinates
//     goal: coordinates to reach
//     threshold: threshold limit to reach goal
//     m_map: modified map considering radius and clearance (??)
//     actions: specified set od actions possible
//     action_costs: costs of actions
//     heuristic: heuristic cost function
//     returns:
//     path: ??
    	
    
//     // code
// }


int main()
{
	float r,c,xs,ys,thetas,xg,yg,thetag;
	std::cout<<"Please input radius and clearance as \"r c\" (without the quotes)\n";
	std::cin>>r>>c;
	std::cout<<
	"Please input starting configuration as \"x y theta\" (without the quotes) where theta is in radians\n";
	std::cin>>xs>>ys>>thetas;
	std::cout<<
	"Please input goal configuration as \"x y theta\" (without the quotes) where theta is in radians\n";
	std::cin>>xg>>yg>>thetag;

    return 0;
}