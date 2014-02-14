
$(function() {

	window.ros = new ROSLIB.Ros();
	window.attitude = new AttitudeIndicator("#attitude");

	ros.on("connection", function() {

		window.muxListener = new ROSLIB.Topic({
			"ros":ros,
			"name":"/mux/selected",
			"messageType":"std_msgs/String"
		});

		window.batteryListener = new ROSLIB.Topic({
			"ros":ros,
			"name":"/ardrone/navdata",
			"messageType":"ardrone_autonomy/Navdata"
		});

		window.leapListener = new ROSLIB.Topic({
			"ros":ros,
			"name":"/leap",
			"messageType":"rosleap/leap"
		});

		leapListener.subscribe(function (msg) {

			var pitch = msg.pitch/1 + 90;
			var roll = msg.roll/(-1);
			attitude.updateAngles(roll,pitch);

			if (window.leapTimeout) {
				clearTimeout(window.leapTimeout);
			}
			window.leapTimeout = setTimeout(function() {
				attitude.updateAngles(0,0);
			}, 500);

		});

		batteryListener.subscribe(function(msg) {

			var b = msg.batteryPercent / 1;
			var color = "green";
			if (b < 50) color = "yellow";
			if (b < 30) color = "red";
			$("#battery").text(b+"%").css("color", color);

			if (window.batteryTimeout) {
				clearTimeout(window.batteryTimeout);
			}
			window.batteryTimeout = setTimeout(function() {
				$("#battery").text("--").css("color", "white");
			}, 500);

		});

		muxListener.subscribe(function(msg) {

			var controls = {
				"wii" : ["wiimote", "red"],
				"leap" : ["leap motion", "green"],
				"ballfollow" : ["autonomous ball following", "yellow"],
				"hooptrick" : ["autonomous hoop trick", "yellow"]
			};
			
			var control = "unknown";
			var color = "white";
			for (var i in controls) {
				if (msg.data.toLowerCase().indexOf(i)>=0) {
					control = controls[i][0];
					color = controls[i][1];
					break;
				}
			}
			
			$("#control").text(control).css("color", color);

		});

	});

	//put flexo's IP here
	ros.connect("ws://localhost:9090");

});
