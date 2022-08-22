
function mask_a_star_search(object_in, target_x, target_y, collision_grid)
{
	// input arguments
	// object_in: id of object,
	// target_x: x coordinate of destination
	// target_y: y coordinate of destination
	// collision_grid: collision grid, created from internal function: mp_grid_create
	
	
	// init grid based on room size and tile size
	var cell_size = TILE_SIZE;                     // cell size or tile size
	var n_cell_width = room_width div cell_size;   // number of cell across room width
	var n_cell_height = room_height div cell_size; // number of cell across room height
	var checked_grid = ds_grid_create(n_cell_width, n_cell_height);
	
	// target points in grid
	var grid_target_x = target_x div TILE_SIZE;
	var grid_target_y = target_y div TILE_SIZE;
		
	// cost to move through empty and obstacle
	var empty_cost = 1;

	// get current object grids points and save it to all object grids
	var grid_object_points = get_object_grid_points(object_in);
	var all_grid_object_points = array_create(0,0);
	array_push(all_grid_object_points, grid_object_points);

	var all_path = array_create(0,0);  
	var all_cost = array_create(0,0);  
	
	// save initial obj x and y
	var obj_x = object_in.x div TILE_SIZE;
	var obj_y = object_in.y div TILE_SIZE;
	var all_obj_x = array_create(0, 0); 
	var all_obj_y = array_create(0, 0);
	// add current obj x and obj y into all obj x and all obj y
	array_push(all_obj_x, obj_x);
	array_push(all_obj_y, obj_y);
	
	// create current path
	var current_path = array_create(0, 0); 
	var current_cost = heuristic_distance(obj_x, obj_y, grid_target_x, grid_target_y);
	array_push(current_path, [obj_x, obj_y]);  // array
	// add current path to all path
	array_push(all_path, current_path);
	array_push(all_cost, current_cost);

	// 1st grid point
	checked_grid[# obj_x, obj_y]= 1;

	// end path and cost
	var end_path = array_create(0,0);  // ds list
	var end_cost = array_create(0,0);  // ds list
	
	var max_search_number = ((RESOLUTION_H/TILE_SIZE) * (RESOLUTION_W/TILE_SIZE)) /1;
	var search_number = 0;
	
	var searched_paths = array_create(0 ,0);  // added for debug purpose
	
	// search recursively
	// the lowest cost always will be at 1st index	
	while (array_length(end_path) == 0) &&     // not reaching target point 
	      (array_length(all_obj_x) > 0) &&     // at least 1 next search point
	      (search_number < max_search_number)  // less than max search number 
	{
		// increase search number
		search_number += 1;
		
		// get obj x and y
		var obj_x = all_obj_x[0];
		var obj_y = all_obj_y[0];
		array_delete(all_obj_x, 0, 1);  // delete 1 number at 0 index position
		array_delete(all_obj_y, 0, 1);
	
		// get current path and cost
		var current_path = all_path[0];
		var current_cost = all_cost[0];
		array_delete(all_path, 0, 1);  // delete 1 number at 0 index position
		array_delete(all_cost, 0, 1);
	
		array_push(searched_paths, current_path);
	
		// get current obj grids
		var grid_object_points = all_grid_object_points[0];
		array_delete(all_grid_object_points, 0, 1);  // delete 1 number at 0 index position
	
		// get current obj each grids
		var grid_top_xs =grid_object_points[0];
		var grid_top_ys = grid_object_points[1];
		var grid_bottom_xs = grid_object_points[2];
		var grid_bottom_ys = grid_object_points[3];
		var grid_left_xs = grid_object_points[4];
		var grid_left_ys = grid_object_points[5];
		var grid_right_xs = grid_object_points[6];
		var grid_right_ys = grid_object_points[7];
	
	
		// search process (generic) -------------------------------------------------------------------------------------------------
	
		var search_directions = ["top", "bottom", "left", "right"];
	
		for (var j=0;j<array_length(search_directions);j++)
		{
			var search_direction = search_directions[j];
	
			if search_direction == "top"
			{
				var next_x = 0;
				var next_y = -1;
				var grid_xs = grid_top_xs;
				var grid_ys = grid_top_ys;
		
			}
			else if search_direction == "bottom"
			{
				var next_x = 0;
				var next_y = 1;
				var grid_xs = grid_bottom_xs;
				var grid_ys = grid_bottom_ys;
			}
			else if search_direction == "left"
			{
				var next_x = -1;
				var next_y = 0;
				var grid_xs = grid_left_xs;
				var grid_ys = grid_left_ys;
			}
			else if search_direction == "right"
			{
				var next_x = 1;
				var next_y = 0;
				var grid_xs = grid_right_xs;
				var grid_ys = grid_right_ys;
			}
	
			// get object next x and next y based on next direction
			var next_obj_x = obj_x+next_x;
			var next_obj_y = obj_y+next_y;
			
			var valid_xy = (next_obj_x < n_cell_width -1) &&
			               (next_obj_x >= 0) &&
						   (next_obj_y < n_cell_height -1) &&
			               (next_obj_y >= 0)
						   
				
			if valid_xy && (checked_grid[# next_obj_x, next_obj_y] == 0)  // grid not check before
			{
				// set grid as checked
				checked_grid[# next_obj_x, next_obj_y] = 1;
		
				// check if border touching obstacle
				var collide_obstacle = 0;
				for (var i=0; i<array_length(grid_xs); i++)
				{
					var grid_x = grid_xs[i] + next_x;
					var grid_y = grid_ys[i] + next_y;
					if (mp_grid_get_cell(collision_grid, grid_x, grid_y) == -1)
					{
						collide_obstacle = 1;
						break;
					}
					
				}
	
				
				if !collide_obstacle
				{	
					// initialize next cost
					var next_cost = 0;
					next_cost += current_cost;
					
					// add manhattan distance as additional cost
					next_cost += heuristic_distance(grid_x, grid_y, grid_target_x, grid_target_y);
					
					// add empty cost
					next_cost += empty_cost;
		
					// create next path 
					var next_path = array_create(0, 0);
					for (var i=0; i< array_length(current_path); i++)
					{
						array_push(next_path, current_path[i]);	
					}
					array_push(next_path, [next_obj_x, next_obj_y]);  // add next obj x and y into next path
			
		
					// next path reach destination
					if next_obj_x == grid_target_x && next_obj_y == grid_target_y
					{
						array_push(end_path, next_path);
						array_push(end_cost, next_cost);
						break;
					}
					// pack new grids, new path and search again
					else 
					{
						// get next object grids 
						var next_grid_object_points = update_grid_points(grid_object_points, search_direction);
				
						// save parameters in ascending order
						var insert_index = -1;  // default value, add at last position
						for (var i=0; i<array_length(all_cost); i++)
						{
							if next_cost <= all_cost[i]
							{
								insert_index = i;  // for adding in min index
								break;
							}
						}
				
						// add new element at last position
						if insert_index == -1
						{
							array_push(all_grid_object_points, next_grid_object_points);
							array_push(all_obj_x, next_obj_x);
							array_push(all_obj_y, next_obj_y);
							array_push(all_path, next_path);
							array_push(all_cost, next_cost);
					
						}
						// add between indexes
						else
						{
							array_insert(all_grid_object_points, insert_index, next_grid_object_points);
							array_insert(all_obj_x, insert_index, next_obj_x);
							array_insert(all_obj_y, insert_index, next_obj_y);
							array_insert(all_path, insert_index, next_path);
							array_insert(all_cost, insert_index, next_cost);	
						}

					}
				}
			}
		}		
	}
	
	// destroy grid
	ds_grid_destroy(checked_grid);
 
	return [end_path, searched_paths];
}


// update object next grid based on direction
function update_grid_points(grid_object_points, update_direction)
{
	if update_direction == "top"
	{
		var next_x = 0;
		var next_y = -1;
	}
	else if update_direction == "bottom"
	{
		var next_x = 0;
		var next_y = 1;
	}
	else if update_direction == "left"
	{
		var next_x = -1;
		var next_y = 0;
	}
	else if update_direction == "right"
	{
		var next_x = 1;
		var next_y = 0;
	}
	
	// get object current grids 
	var grid_top_xs = grid_object_points[0];
	var grid_top_ys = grid_object_points[1];
	var grid_bottom_xs = grid_object_points[2];
	var grid_bottom_ys = grid_object_points[3];
	var grid_left_xs = grid_object_points[4];
	var grid_left_ys = grid_object_points[5];
	var grid_right_xs = grid_object_points[6];
	var grid_right_ys = grid_object_points[7];
				
	// update grids
	// top
	var next_grid_top_xs = array_create(0, 0);
	var next_grid_top_ys = array_create(0, 0);
	for (var i=0; i<array_length(grid_top_xs); i++)
	{
		// get current grid
		var grid_x = grid_top_xs[i];
		var grid_y = grid_top_ys[i];
		// save next grid
		array_push(next_grid_top_xs, grid_x+next_x);
		array_push(next_grid_top_ys, grid_y+next_y);
	}
	// bottom
	var next_grid_bottom_xs = array_create(0, 0);
	var next_grid_bottom_ys = array_create(0, 0);
	for (var i=0; i<array_length(grid_bottom_xs); i++)
	{
		// get current grid
		var grid_x = grid_bottom_xs[i];
		var grid_y = grid_bottom_ys[i];
		// save next grid
		array_push(next_grid_bottom_xs, grid_x+next_x);
		array_push(next_grid_bottom_ys, grid_y+next_y);
	}
	// left
	var next_grid_left_xs = array_create(0, 0);
	var next_grid_left_ys = array_create(0, 0);
	for (var i=0; i<array_length(grid_left_xs); i++)
	{
		// get current grid
		var grid_x = grid_left_xs[i];
		var grid_y = grid_left_ys[i];
		// save next grid
		array_push(next_grid_left_xs, grid_x+next_x);
		array_push(next_grid_left_ys, grid_y+next_y);
	}
	// right
	var next_grid_right_xs = array_create(0, 0);
	var next_grid_right_ys = array_create(0, 0);
	for (var i=0; i<array_length(grid_right_xs); i++)
	{
		// get current grid
		var grid_x = grid_right_xs[i];
		var grid_y = grid_right_ys[i];
		// save next grid
		array_push(next_grid_right_xs, grid_x+next_x);
		array_push(next_grid_right_ys, grid_y+next_y);
	}
				
	// update grid object points
	var next_grid_points = array_create(0, 0);
				
	// save grid points
	array_push(next_grid_points, next_grid_top_xs)	
	array_push(next_grid_points, next_grid_top_ys);
	array_push(next_grid_points, next_grid_bottom_xs);
	array_push(next_grid_points, next_grid_bottom_ys);
	array_push(next_grid_points, next_grid_left_xs);
	array_push(next_grid_points, next_grid_left_ys);
	array_push(next_grid_points, next_grid_right_xs);
	array_push(next_grid_points, next_grid_right_ys);

	return next_grid_points
}

// get object grid points based on their bbox borders
function get_object_grid_points(object_in)
{
	var grid_points = array_create(0,0); 
	var grid_top_xs = array_create(0,0);
	var grid_top_ys = array_create(0,0);
	var grid_bottom_xs = array_create(0,0);
	var grid_bottom_ys = array_create(0,0);
	var grid_left_xs = array_create(0,0);
	var grid_left_ys = array_create(0,0);
	var grid_right_xs = array_create(0,0);
	var grid_right_ys = array_create(0,0);

	// top
	var grid_top_y = object_in.bbox_top div TILE_SIZE;
	// bottom
	var grid_bottom_y = object_in.bbox_bottom div TILE_SIZE;
	// left
	var grid_left_x = object_in.bbox_left div TILE_SIZE;
	// right
	var grid_right_x = object_in.bbox_right div TILE_SIZE;
	
	// top and bottom row
	var x_end= 0;
	for (var cx=object_in.bbox_left; cx<=object_in.bbox_right; cx+=TILE_SIZE)
	{
		// get grid point
		var grid_x = cx div TILE_SIZE;
		// save grid points
		// top
		array_push(grid_top_xs, grid_x);
		array_push(grid_top_ys, grid_top_y);
		// bottom
		array_push(grid_bottom_xs, grid_x);
		array_push(grid_bottom_ys, grid_bottom_y);
		
		// last point
		if (cx + TILE_SIZE >= object_in.bbox_right) && !x_end 
		{
			cx = object_in.bbox_right - TILE_SIZE;
			x_end = 1;	
		}
	}
	
	// left and right column
	var y_end= 0;
	for (var cy=object_in.bbox_top; cy<=object_in.bbox_bottom; cy+=TILE_SIZE)
	{
		// get grid point
		var grid_y = cy div TILE_SIZE;
		// save grid points
		// left
		array_push(grid_left_xs, grid_left_x);
		array_push(grid_left_ys, grid_y);
		// right
		array_push(grid_right_xs, grid_right_x);
		array_push(grid_right_ys, grid_y);
		
		// last point
		if (cy + TILE_SIZE >= object_in.bbox_bottom) && !y_end 
		{
			cy = object_in.bbox_bottom - TILE_SIZE;
			y_end = 1;	

		}	
	}
	
	// save grid points
	array_push(grid_points, grid_top_xs)	
	array_push(grid_points, grid_top_ys);
	array_push(grid_points, grid_bottom_xs);
	array_push(grid_points, grid_bottom_ys);
	array_push(grid_points, grid_left_xs);
	array_push(grid_points, grid_left_ys);
	array_push(grid_points, grid_right_xs);
	array_push(grid_points, grid_right_ys);
	
	return grid_points
}

// manhattan distance
function heuristic_distance(cx, cy, tx, ty)
{
	return abs(ty-cy) + abs(tx-cx)
}
