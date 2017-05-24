# Test cases communication

#### 1. Normal communication get_status
  * get_status : once every 0.5s

#### 2. Error communication get_status
  * get_status : once every 0.5s
  * **bad get_status** : once every 10s

#### 3. Normal communication move_motor
  * get_status : once every 0.5s
  * move_motor x and y : once every 10s
  
#### 4. Error communication move_motor
  * get_status : once every 0.5s
  * move_motor x and y : once every 10s
  * **bad move_motor** x and y : once every 20s
  
#### 5. Normal communication homing
  * get_status : once every 0.5s
  * homing : once every 5 min
  
#### 6. Error communication move_motor
  * get_status : once every 0.5s
  * homing : once every 5 min
  * **bad homing** : once every 6 min
  
#### 7. Move while homing
  * get_status : once every 0.5s
  * homing : once every 5 min
  * move_motor: while homing
