# Highway Path Planning

Based on the introduction tutorial in the udacity project the lane following/change and speed up was implemented with the spline.h library.

Other improvements to fit the path planning were added:

## Speed management

Speed change is determined in respect of cars in front of the vehicle. The speed is clipped to not speed above the speed limit.

## Detection of other cars

Based on the sensor data for each car detected following status parameters will be calculated:

* to_close --> A car is in front of the vehicle
* car_left --> Car beside left of the vehicle 
* car_right --> Car beside right of the vehicle
* car_left_speed --> Speed of possible left car 
* car_right_speed --> Speed of possible right car

### Car in front:
A vehicle s is greater than the car's s, and it is on the same lane

### Car left/right
A car is on the side of the vehicle if it is within a range before the vehicle or a specific range behind it. Because the car must not collide when switching lanes, the range behind is as big as the car needs to switch the lane. The range infront is greater to determine the best lane change based on speed. 

    ||| --> Respected zone to check for cars
____________________________________________
            |||||||||||||||||||||||||||||| LANE 1 
____________________________________________
             Car [XXX] ||||||||||||||||||| LANE 2
____________________________________________
            |||||||||||||||||||||||||||||| LANE 3
____________________________________________

### Car speed 
The speed of a car in front (left/right) is stored, if no car is present the speed is set to a MAX_SPEED value

## Lane Change model

    If a vehicle is to close
        if we are in the mid of the road and all other lanes are free 
            switch lane based on the speed of cars in front, take the lane with the faster or no car
        else if we are not on the left side of the road
            try to change lane to the left
        else if we are not on the right side of the road
            try to change lane to the right 
        speed down the car to match speed of car in front of my lane
    else
        speed up to safe speed margin below speed limit