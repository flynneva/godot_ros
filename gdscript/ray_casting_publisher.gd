extends Spatial

var ros_raycast_publisher = RayCastPublisher.new()

var sensor_array = []



const RaySensorResource = preload("res://scenes/RaySensor.tscn")
var ray_sensor_array = []

var sensor_period = 0.1
var last_update_time = 0.0

export var MAX_ROW = 20
export var MAX_COL = 20

var num_points = MAX_ROW * MAX_COL
var sensor_interspace = 0.05

export var H_ANGLE_OP = 20.0
export var V_ANGLE_OP = 20.0

var HORIZONTAL_OPERTURE_ANGLE =  deg2rad(H_ANGLE_OP)
var delta_h = 2*HORIZONTAL_OPERTURE_ANGLE / (MAX_COL-1)


var VERTICAL_OPERTURE_ANGLE = deg2rad(V_ANGLE_OP)
var delta_v = 2*VERTICAL_OPERTURE_ANGLE / (MAX_ROW - 1)

# Called when the node enters the scene tree for the first time.
func _ready():
	ros_raycast_publisher.spin_some()
	create_ray_matrix()

func create_ray_matrix():
	

	
	var row_num = 1
	var col_num = 1
	var number_of_sensors = MAX_ROW * MAX_COL
	
	var horizontal_angle = -1*HORIZONTAL_OPERTURE_ANGLE
	var vertical_angle =-1*VERTICAL_OPERTURE_ANGLE
	
	for i in range(number_of_sensors):
		
		
		var RaySensorInstance= RaySensorResource.instance()
		#You could now make changes to the new instance if you wanted
		RaySensorInstance.name = "RaySensor"+String(i)
		#Attach it to the tree
		var vector = Vector3(col_num * sensor_interspace, row_num * sensor_interspace, 0.0)
		
		RaySensorInstance.rotate_y(horizontal_angle)
		RaySensorInstance.rotate_x(vertical_angle)
		self.add_child(RaySensorInstance)
		ray_sensor_array.append(RaySensorInstance)
		
		if col_num >= MAX_COL:
			col_num = 1
			row_num += 1
			
			if vertical_angle  > VERTICAL_OPERTURE_ANGLE:
				vertical_angle = -1*VERTICAL_OPERTURE_ANGLE
			else:
				vertical_angle += delta_v
			
			
			if row_num > MAX_ROW:
				break
		else:
			col_num += 1
			
			if horizontal_angle  >= HORIZONTAL_OPERTURE_ANGLE:
				horizontal_angle = -1*HORIZONTAL_OPERTURE_ANGLE
			else:
				horizontal_angle += delta_h
		
	
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func update_sensordata():
	sensor_array = []
	for sensor in ray_sensor_array:
		var col_point = Vector3()
		if sensor.collides():
			sensor.update_hitpoints()
			col_point = sensor.get_col_point()
		else:
			sensor.hide_hitpoint()
		
		sensor_array.append(col_point)
	
	ros_raycast_publisher.publish_raycast_msg(sensor_array)

func _process(delta):
	
	if last_update_time >= sensor_period:
		update_sensordata()
		last_update_time = 0.0
	else:
		last_update_time += delta
