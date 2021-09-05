extends Spatial

signal timeout
var g_timer = null
var count = 1

var ros_viewport = ViewPort.new()

# Called when the node enters the scene tree for the first time.
func _ready():

	# spin node
	ros_viewport.spin_some()
	
	g_timer = Timer.new()
	add_child(g_timer)
	# connect the talk signal with the talk callback
	g_timer.connect("timeout", self, "_on_Timer_timeout")
	g_timer.set_wait_time(1.0)
	g_timer.set_one_shot(false)
	g_timer.start()
	pass # Replace with function body.

func _on_Timer_timeout():
	print("godot viewport publishing" + String(count))
	get_viewport().set_clear_mode(Viewport.CLEAR_MODE_ONLY_NEXT_FRAME)
	# Wait until the frame has finished before getting the texture.
	yield(VisualServer, "frame_post_draw")
	# Retrieve the captured image.
	var img = get_viewport().get_texture().get_data()
	img.convert(Image.FORMAT_RGB8)
	ros_viewport.pubImage(img)
	count += 1