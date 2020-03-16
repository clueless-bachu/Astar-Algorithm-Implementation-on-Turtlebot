get_children(state, actions, action_costs, m_map, stepsize, theta)
{
	/*  Explores the child nodes
    input:
    state: current coordinates
    actions: specified set od actions possible
    action_costs: costs of actions
    m_map: modified map considering radius and clearance (??)
    stepsize: stepsize of movement of radius
    theta: angle between action set at each node
    returns:
    children: dictionary which maps the child coordinates to cost (??)
    total_cost: total cost of the child node (??)
    */
    
    // code
}

state_validity(state, m_map)
{
	/*
	Checks whether the state is inside the obstacle space
	input:
	state: current coordinates
	m_map: modified map considering radius and clearance (??)
	returns:
	a boolean value
	*/
	
	// code
}

m_map(map, radius, clearance)
{
	/*
	Gives modified obstacle space
	input:
	map: original obstacle space
	radius: radius of rigid robot
	clearance: clearance of rigid robot
	returns:
	m_map: modified map considering radius and clearance (??)
	*/
	
	// code	
}

A_star(start, goal, threshold, m_map, actions, action_costs, heuristic)
{
	/*  Explores the child nodes
    input:
    start: current coordinates
    goal: coordinates to reach
    threshold: threshold limit to reach goal
    m_map: modified map considering radius and clearance (??)
    actions: specified set od actions possible
    action_costs: costs of actions
    heuristic: heuristic cost function
    returns:
    path: ??
    */	
    
    // code
}


