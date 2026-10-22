extends Node3D
class_name Smoke

@export_group("Shape")
@export var cell_size: float = 0.3
@export var gap: float = 0.1
@export var radius: float = 3.0
@export_range(0.1, 3.0) var volume_ratio: float = 1.0
@export var max_extra_reach: float = 100.0
@export var diagonal_spread: bool = true

@export_group("Motion")
@export var variation: float = 0.2
@export var variation_speed: float = 1.2

@export_group("Durations")
@export var grow_duration: float = 1.0
@export var lifetime: float = 5.0
@export var fade_duration: float = 1.5

@export_group("Collisions")
@export_flags_3d_physics var collision_mask: int = 0xFFFFFFFF
@export var excluded_bodies: Array[CollisionObject3D] = []
@export var wall_margin: float = 0.0
@export var line_of_sight: bool = false

@export_group("Rendering")
@export var debug: bool = false
@export var particles: GPUParticles3D
@export var particle_update_interval: float = 0.1
@export var particles_per_cell: int = 4

enum State { IDLE, GROWING, HOLDING, FADING }

const DIRS_6: Array[Vector3i] = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1),
]

var _state: State = State.IDLE
var _time: float = 0.0
var _life_time: float = 0.0

var _positions: PackedVector3Array = PackedVector3Array()
var _keys: PackedFloat32Array = PackedFloat32Array()
var _max_key: float = 0.0
var _extent: float = 0.0
var _count: int = 0

var _mmi: MultiMeshInstance3D
var _particle_mat: ParticleProcessMaterial
var _particle_timer: float = 0.0
var _last_n: int = -1

var _space: PhysicsDirectSpaceState3D
var _origin: Vector3
var _rmax: float
var _shape_q: PhysicsShapeQueryParameters3D
var _ray_q: PhysicsRayQueryParameters3D
var _los_q: PhysicsRayQueryParameters3D
var _free_cache := {}
var _neighbors: Array[Vector3i] = []
var _hk: Array[float] = []
var _hv: Array[Vector3i] = []
var _pop_key: float = 0.0

var _step: float:
	get: return cell_size + gap


func _ready() -> void:
	if particles:
		particles.visible = true
		particles.emitting = false


func is_alive() -> bool:
	return _state != State.IDLE


func start() -> void:
	if is_alive():
		return

	if not _compute_cells():
		return

	_count = _positions.size()

	if debug:
		_mmi = _make_multimesh()
		if particles:
			particles.emitting = false
	else:
		_start_particles()

	_time = 0.0
	_life_time = 0.0
	_last_n = -1
	_particle_timer = particle_update_interval
	_state = State.GROWING


func _build_neighbors() -> void:
	_neighbors.clear()
	if not diagonal_spread:
		_neighbors.assign(DIRS_6)
		return
	for x in range(-1, 2):
		for y in range(-1, 2):
			for z in range(-1, 2):
				if x != 0 or y != 0 or z != 0:
					_neighbors.append(Vector3i(x, y, z))


func _compute_cells() -> bool:
	_space = get_world_3d().direct_space_state
	_origin = global_position
	_rmax = radius + variation

	var excluded: Array[RID] = []
	for b in excluded_bodies:
		if b:
			excluded.append(b.get_rid())

	var box := BoxShape3D.new()
	box.size = Vector3.ONE * (_step * 1.02 + wall_margin * 2.0)
	_shape_q = PhysicsShapeQueryParameters3D.new()
	_shape_q.shape = box
	_shape_q.collision_mask = collision_mask
	_shape_q.exclude = excluded

	_ray_q = PhysicsRayQueryParameters3D.new()
	_ray_q.collision_mask = collision_mask
	_ray_q.exclude = excluded
	_ray_q.hit_from_inside = true
	_ray_q.hit_back_faces = true

	_los_q = PhysicsRayQueryParameters3D.new()
	_los_q.collision_mask = collision_mask
	_los_q.exclude = excluded
	_los_q.hit_from_inside = true
	_los_q.hit_back_faces = true

	_free_cache.clear()
	_hk.clear()
	_hv.clear()
	_build_neighbors()

	if not _free(Vector3i.ZERO):
		return false

	var sphere_volume := (4.0 / 3.0) * PI * pow(radius, 3.0)
	var budget := maxi(int(sphere_volume / pow(_step, 3.0) * volume_ratio), 1)
	var reach_cap := radius + max_extra_reach

	var dist := {}
	var closed := {}
	var cells: Array[Vector3i] = []
	var cell_keys: Array[float] = []

	dist[Vector3i.ZERO] = 0.0
	_hpush(0.0, Vector3i.ZERO)

	while not _hk.is_empty() and cells.size() < budget:
		var c := _hpop()
		var k := _pop_key
		if closed.has(c):
			continue
		if k > reach_cap:
			break
		closed[c] = true
		cells.append(c)
		cell_keys.append(k)

		for d in _neighbors:
			var n := c + d
			if closed.has(n):
				continue
			var nd := k + Vector3(d).length() * _step
			if dist.has(n) and dist[n] <= nd:
				continue
			if not _free(n):
				continue
			if _wall_between(c, n):
				continue
			dist[n] = nd
			_hpush(nd, n)

	_positions.resize(cells.size())
	_keys.resize(cells.size())
	_extent = 0.0
	for i in cells.size():
		var p := Vector3(cells[i]) * _step
		_positions[i] = p
		_keys[i] = cell_keys[i]
		_extent = maxf(_extent, maxf(absf(p.x), maxf(absf(p.y), absf(p.z))))
	_max_key = _keys[_keys.size() - 1] if not _keys.is_empty() else 0.0

	_free_cache.clear()
	_hk.clear()
	_hv.clear()
	return not _positions.is_empty()


func _free(c: Vector3i) -> bool:
	if _free_cache.has(c):
		return _free_cache[c]

	var ok := true
	_shape_q.transform = Transform3D(Basis.IDENTITY, _origin + Vector3(c) * _step)
	if not _space.intersect_shape(_shape_q, 1).is_empty():
		ok = false
	elif line_of_sight and c != Vector3i.ZERO:
		_los_q.from = _origin
		_los_q.to = _origin + Vector3(c) * _step
		if not _space.intersect_ray(_los_q).is_empty():
			ok = false

	_free_cache[c] = ok
	return ok


func _wall_between(a: Vector3i, b: Vector3i) -> bool:
	_ray_q.from = _origin + Vector3(a) * _step
	_ray_q.to = _origin + Vector3(b) * _step
	return not _space.intersect_ray(_ray_q).is_empty()


func _hpush(k: float, v: Vector3i) -> void:
	_hk.append(k)
	_hv.append(v)
	var i := _hk.size() - 1
	while i > 0:
		var p := (i - 1) >> 1
		if _hk[p] <= _hk[i]:
			break
		var tk := _hk[p]; _hk[p] = _hk[i]; _hk[i] = tk
		var tv := _hv[p]; _hv[p] = _hv[i]; _hv[i] = tv
		i = p


func _hpop() -> Vector3i:
	var top := _hv[0]
	_pop_key = _hk[0]
	var lk: float = _hk.pop_back()
	var lv: Vector3i = _hv.pop_back()
	var n := _hk.size()
	if n > 0:
		_hk[0] = lk
		_hv[0] = lv
		var i := 0
		while true:
			var l := i * 2 + 1
			var r := l + 1
			var m := i
			if l < n and _hk[l] < _hk[m]:
				m = l
			if r < n and _hk[r] < _hk[m]:
				m = r
			if m == i:
				break
			var tk := _hk[m]; _hk[m] = _hk[i]; _hk[i] = tk
			var tv := _hv[m]; _hv[m] = _hv[i]; _hv[i] = tv
			i = m
	return top


func _make_multimesh() -> MultiMeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3.ONE * cell_size

	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color.WHITE
	mesh.material = mat

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = mesh
	mm.instance_count = _positions.size()
	for i in _positions.size():
		mm.set_instance_transform(i, Transform3D(Basis.IDENTITY, _origin + _positions[i]))
	mm.visible_instance_count = 0

	var inst := MultiMeshInstance3D.new()
	inst.multimesh = mm
	inst.top_level = true
	inst.global_transform = Transform3D.IDENTITY
	add_child(inst)
	return inst


func _start_particles() -> void:
	if particles == null:
		return

	var src := particles.process_material as ParticleProcessMaterial
	if src == null:
		push_warning("Smoke: the emitter must have a ParticleProcessMaterial.")
		return

	if _particle_mat == null:
		_particle_mat = src.duplicate() as ParticleProcessMaterial
		particles.process_material = _particle_mat
	_particle_mat.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_POINTS

	particles.visible = true
	particles.one_shot = false
	particles.explosiveness = 0.0
	particles.amount = maxi(_positions.size() * maxi(particles_per_cell, 1), 1)
	particles.top_level = true
	particles.local_coords = true
	particles.global_transform = Transform3D(Basis.IDENTITY, global_position)

	var reach := _extent + _step * 2.0
	particles.visibility_aabb = AABB(Vector3.ONE * -reach, Vector3.ONE * reach * 2.0)

	_update_particle_points(1)
	particles.restart()
	particles.emitting = true


func _update_particle_points(n: int) -> void:
	n = maxi(n, 1)
	var w := 128
	var h := int(ceil(float(n) / w))
	var floats := PackedFloat32Array()
	floats.resize(w * h * 3)
	for i in w * h:
		var p := _positions[mini(i, n - 1)]
		floats[i * 3] = p.x
		floats[i * 3 + 1] = p.y
		floats[i * 3 + 2] = p.z

	var img := Image.create_from_data(w, h, false, Image.FORMAT_RGBF, floats.to_byte_array())
	_particle_mat.emission_point_texture = ImageTexture.create_from_image(img)
	_particle_mat.emission_point_count = n
	particles.amount_ratio = clampf(float(n) / float(maxi(_positions.size(), 1)), 0.0, 1.0)


func _live_radius() -> float:
	var t := _life_time * variation_speed
	var s := (sin(t * 1.7) + sin(t * 2.9 + 1.3)) * 0.5
	return radius + variation * s


func _count_within(d: float) -> int:
	if d <= 0.0:
		return 0
	var lo := 0
	var hi := _keys.size()
	while lo < hi:
		var mid := (lo + hi) >> 1
		if _keys[mid] <= d:
			lo = mid + 1
		else:
			hi = mid
	return lo


func _apply_ratio(ratio: float, delta: float) -> void:
	var n := _keys.size() if ratio >= 1.0 else _count_within(_max_key * ratio)
	if debug:
		if _mmi:
			_mmi.multimesh.visible_instance_count = n
	elif particles and _particle_mat:
		_particle_timer += delta
		if _particle_timer >= particle_update_interval and n != _last_n:
			_particle_timer = 0.0
			_last_n = n
			_update_particle_points(n)


func _process(delta: float) -> void:
	if _state == State.IDLE:
		return

	_time += delta
	_life_time += delta
	var live_ratio := _live_radius() / _rmax

	match _state:
		State.GROWING:
			var t := clampf(_time / grow_duration, 0.0, 1.0)
			_apply_ratio(live_ratio * smoothstep(0.0, 1.0, t), delta)
			if t >= 1.0:
				_time = 0.0
				_state = State.HOLDING

		State.HOLDING:
			_apply_ratio(live_ratio, delta)
			if _time >= lifetime:
				_time = 0.0
				_state = State.FADING

		State.FADING:
			var t := clampf(_time / fade_duration, 0.0, 1.0)
			_apply_ratio(live_ratio * (1.0 - t), delta)
			if t >= 1.0:
				_end()


func _end() -> void:
	if _mmi:
		_mmi.queue_free()
		_mmi = null
	if particles:
		particles.emitting = false
	_positions.clear()
	_keys.clear()
	_max_key = 0.0
	_extent = 0.0
	_count = 0
	_state = State.IDLE
