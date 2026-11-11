extends Node

signal state_received(state: Dictionary)

@export var port: int = 9999

var packets_total: int = 0
var parse_errors: int = 0
var packet_rate: float = 0.0
var listening: bool = false

var _server := UDPServer.new()
var _peer: PacketPeerUDP = null
var _last_packet_msec: int = -1
var _rate_count: int = 0
var _rate_timer: float = 0.0


func _ready() -> void:
	var err := _server.listen(port)
	listening = err == OK
	if listening:
		print("[udp] listening on udp://*:%d" % port)
	else:
		push_error("[udp] could not listen on port %d (error %d)" % [port, err])


func _process(delta: float) -> void:
	if not listening:
		return
	_server.poll()
	while _server.is_connection_available():
		_peer = _server.take_connection()

	if _peer != null:
		var latest := PackedByteArray()
		while _peer.get_available_packet_count() > 0:
			latest = _peer.get_packet()
			packets_total += 1
			_rate_count += 1
		if latest.size() > 0:
			var data = JSON.parse_string(latest.get_string_from_utf8())
			if data is Dictionary:
				_last_packet_msec = Time.get_ticks_msec()
				state_received.emit(data)
			else:
				parse_errors += 1

	_rate_timer += delta
	if _rate_timer >= 0.5:
		packet_rate = _rate_count / _rate_timer
		_rate_count = 0
		_rate_timer = 0.0


func seconds_since_packet() -> float:
	if _last_packet_msec < 0:
		return INF
	return (Time.get_ticks_msec() - _last_packet_msec) / 1000.0


func _exit_tree() -> void:
	_server.stop()
