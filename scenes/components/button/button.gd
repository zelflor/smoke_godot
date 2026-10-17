
extends StaticBody3D

@export var target_node: Node


func interact() -> void:
	if not is_instance_valid(target_node):
		push_warning("No target node assigned to %s." % name)
		return

	if not target_node.has_method("start"):
		push_warning(
			"Target node '%s' does not have a start() function."
			% target_node.name
		)
		return

	target_node.call("start")
