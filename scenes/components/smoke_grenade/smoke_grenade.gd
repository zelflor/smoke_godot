extends Node3D
class_name Smoke

@export_group("Forme")
@export var cell_size: float = 0.3
@export var gap: float = 0.1
@export var radius: float = 3.0
@export_range(0.0, 2.0) var overflow_ratio: float = 1.0
@export var max_extra_reach: float = 0.0

@export_group("Mouvement")
@export var variation: float = 0.2
@export var variation_speed: float = 1.2

@export_group("Durées")
@export var grow_duration: float = 1.0
@export var lifetime: float = 5.0
@export var fade_duration: float = 1.5

@export_group("Collisions")
@export_flags_3d_physics var collision_mask: int = 0xFFFFFFFF
@export var excluded_bodies: Array[CollisionObject3D] = []
@export var wall_margin: float = 0.0
@export var line_of_sight: bool = true

@export_group("Rendu")
@export var debug: bool = false
@export var particles: GPUParticles3D
@export var particle_update_interval: float = 0.1
@export var particles_per_cell: int = 4

enum State { IDLE, GROWING, HOLDING, FADING }

const DIRS: Array[Vector3i] = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1),
]

var _state: State = State.IDLE
var _time: float = 0.0
var _life_time: float = 0.0

var _positions: PackedVector3Array = PackedVector3Array()
var _keys: PackedFloat32Array = PackedFloat32Array()
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
var _visited := {}
var _over_seen := {}
var _queue: Array[Vector3i] = []
var _phase2 := false
var _hk: Array[float] = []
var _hv: Array[Vector3i] = []

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
	_visited.clear()
	_over_seen.clear()
	_queue.clear()
	_hk.clear()
	_hv.clear()
	_phase2 = false

	var all_dists := PackedFloat32Array()
	var n_max := int(ceil(_rmax / _step))
	for x in range(-n_max, n_max + 1):
		for y in range(-n_max, n_max + 1):
			for z in range(-n_max, n_max + 1):
				var d := Vector3(x, y, z).length() * _step
				if d <= _rmax:
					all_dists.append(d)
	all_dists.sort()

	if not _free(Vector3i.ZERO):
		return false

	var sphere: Array[Vector3i] = []
	_queue.append(Vector3i.ZERO)
	_visited[Vector3i.ZERO] = true
	var head := 0
	while head < _queue.size():
		var c: Vector3i = _queue[head]
		head += 1
		sphere.append(c)
		_explore(c)

	sphere.sort_custom(func(a: Vector3i, b: Vector3i): return Vector3(a).length_squared() < Vector3(b).length_squared())

	var sphere_dists := PackedFloat32Array()
	for c in sphere:
		sphere_dists.append(Vector3(c).length() * _step)
	sphere_dists.sort()

	var missing_dists := PackedFloat32Array()
	var j := 0
	for d in all_dists:
		if j < sphere_dists.size() and is_equal_approx(sphere_dists[j], d):
			j += 1
		else:
			missing_dists.append(d)

	var budget := int(round(missing_dists.size() * overflow_ratio))
	var reach_max := _rmax + max_extra_reach
	var extras: Array[Vector3i] = []
	_phase2 = true
	while budget > 0 and not _hk.is_empty():
		var c := _hpop()
		if Vector3(c).length() * _step > reach_max:
			continue
		if not _is_corner(c):
			continue
		extras.append(c)
		budget -= 1
		_explore(c)

	var all_cells: Array[Vector4] = []
	for c in sphere:
		var p := Vector3(c) * _step
		all_cells.append(Vector4(p.x, p.y, p.z, p.length()))
	for i in extras.size():
		var p := Vector3(extras[i]) * _step
		var key: float = missing_dists[mini(i, missing_dists.size() - 1)] if not missing_dists.is_empty() else p.length()
		all_cells.append(Vector4(p.x, p.y, p.z, key))

	all_cells.sort_custom(func(a: Vector4, b: Vector4): return a.w < b.w)

	_positions.resize(all_cells.size())
	_keys.resize(all_cells.size())
	for i in all_cells.size():
		var v := all_cells[i]
		_positions[i] = Vector3(v.x, v.y, v.z)
		_keys[i] = v.w

	_free_cache.clear()
	_visited.clear()
	_over_seen.clear()
	_queue.clear()
	_hk.clear()
	_hv.clear()
	return not _positions.is_empty()


func _explore(c: Vector3i) -> void:
	var adj := false
	var pending: Array[Vector3i] = []

	for d in DIRS:
		var n := c + d
		if _visited.has(n):
			if not _free_cache.get(n, true):
				adj = true
			continue
		if _over_seen.has(n):
			continue
		if not _free(n):
			_visited[n] = true
			adj = true
			continue
		if _wall_between(c, n):
			adj = true
			continue
		if not _phase2 and Vector3(n).length() * _step <= _rmax:
			_visited[n] = true
			_queue.append(n)
		else:
			pending.append(n)

	if adj:
		for n in pending:
			if not _over_seen.has(n):
				_over_seen[n] = true
				_hpush(Vector3(n).length_squared(), n)


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


func _is_corner(c: Vector3i) -> bool:
	var blocked_axes := 0
	for axis in 3:
		var axis_blocked := false
		for sgn in [1, -1]:
			var d := Vector3i.ZERO
			d[axis] = sgn
			var n := c + d
			if not _free(n) or _wall_between(c, n):
				axis_blocked = true
				break
		if axis_blocked:
			blocked_axes += 1
	return blocked_axes >= 2


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
		push_warning("Smoke: l'emitter doit avoir un ParticleProcessMaterial.")
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

	var reach := _rmax + max_extra_reach + _step * 2.0
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


func _count_within(r: float) -> int:
	if r <= 0.0:
		return 0
	var lo := 0
	var hi := _keys.size()
	while lo < hi:
		var mid := (lo + hi) >> 1
		if _keys[mid] <= r:
			lo = mid + 1
		else:
			hi = mid
	return lo


func _apply_radius(r: float, delta: float) -> void:
	var n := _count_within(r)
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
	var live := _live_radius()

	match _state:
		State.GROWING:
			var t := clampf(_time / grow_duration, 0.0, 1.0)
			_apply_radius(live * smoothstep(0.0, 1.0, t), delta)
			if t >= 1.0:
				_time = 0.0
				_state = State.HOLDING

		State.HOLDING:
			_apply_radius(live, delta)
			if _time >= lifetime:
				_time = 0.0
				_state = State.FADING

		State.FADING:
			var t := clampf(_time / fade_duration, 0.0, 1.0)
			_apply_radius(live * (1.0 - t), delta)
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
	_count = 0
	_state = State.IDLE
