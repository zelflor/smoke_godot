
extends CharacterBody3D

@onready var head: Node3D = $Head
@onready var camera: Camera3D = $Head/Camera3D

@export_group("Movement")
@export var move_speed: float = 5.0
@export var jump_velocity: float = 4.5
@export var mouse_sensitivity: float = 0.002

@export_group("Camera")
@export_range(-89.0, 0.0, 1.0) var min_pitch: float = -89.0
@export_range(0.0, 89.0, 1.0) var max_pitch: float = 89.0

@export_group("Interaction")
@export var interaction_distance: float = 3.0
@export var interaction_collision_mask: int = 1

var gravity: float = float(
	ProjectSettings.get_setting("physics/3d/default_gravity")
)


func _ready() -> void:
	head.position.y = 0.5
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey:
		if event.pressed and event.keycode == KEY_ESCAPE:
			Input.mouse_mode = Input.MOUSE_MODE_VISIBLE

	if event is InputEventMouseButton:
		if (
			event.pressed
			and event.button_index == MOUSE_BUTTON_LEFT
			and Input.mouse_mode == Input.MOUSE_MODE_VISIBLE
		):
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

	# Bloquer la caméra et les interactions lorsque la souris est libre.
	if Input.mouse_mode != Input.MOUSE_MODE_CAPTURED:
		return

	if event is InputEventMouseMotion:
		rotate_y(-event.relative.x * mouse_sensitivity)

		head.rotate_x(-event.relative.y * mouse_sensitivity)
		head.rotation.x = clampf(
			head.rotation.x,
			deg_to_rad(min_pitch),
			deg_to_rad(max_pitch)
		)

	if (
		event.is_action_pressed("interaction")
		and not event.is_echo()
	):
		interact_with_target()




func _physics_process(delta: float) -> void:
	if Input.mouse_mode != Input.MOUSE_MODE_CAPTURED:
		velocity.x = 0.0
		velocity.z = 0.0

		if not is_on_floor():
			velocity.y -= gravity * delta
		else:
			velocity.y = 0.0

		move_and_slide()
		return

	if not is_on_floor():
		velocity.y -= gravity * delta

	if Input.is_action_just_pressed("jump") and is_on_floor():
		velocity.y = jump_velocity

	var input_direction: Vector2 = Input.get_vector(
		"move_left",
		"move_right",
		"move_forward",
		"move_backward"
	)

	var direction: Vector3 = (
		transform.basis * Vector3(
			input_direction.x,
			0.0,
			input_direction.y
		)
	).normalized()

	if direction.length_squared() > 0.0:
		velocity.x = direction.x * move_speed
		velocity.z = direction.z * move_speed
	else:
		velocity.x = move_toward(velocity.x, 0.0, move_speed)
		velocity.z = move_toward(velocity.z, 0.0, move_speed)

	move_and_slide()


func interact_with_target() -> void:
	var space_state: PhysicsDirectSpaceState3D = (
		get_world_3d().direct_space_state
	)

	var ray_start: Vector3 = camera.global_position
	var ray_end: Vector3 = (
		ray_start
		- camera.global_basis.z * interaction_distance
	)

	var query: PhysicsRayQueryParameters3D = (
		PhysicsRayQueryParameters3D.create(ray_start, ray_end)
	)

	query.collision_mask = interaction_collision_mask
	query.exclude = [get_rid()]

	var result: Dictionary = space_state.intersect_ray(query)

	if result.is_empty():
		return

	var target: Object = result["collider"]

	if target.has_method("interact"):
		target.call("interact")
