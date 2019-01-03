console.log('Client-side code running');

var count = 0;

$(document).keydown(function(e){
	switch(e.keyCode){
		case 87:
			document.getElementById("W").style.backgroundColor = "green";
		    $.post("http://192.168.1.139/forward", "forward");
		    break;
		case 83:
			document.getElementById("S").style.backgroundColor = "green";
			$.post("http://192.168.1.139/backward", "backward");
			break;
		case 65:
			document.getElementById("A").style.backgroundColor = "green";
			$.post("http://192.168.1.139/left", "left");
			break;
		case 68:
			document.getElementById("D").style.backgroundColor = "green";
			$.post("http://192.168.1.139/right", "right");
			break;
	}
});

$(document).keyup(function(e){
	$.post("http://192.168.1.139/stop", "stop");
	switch(e.keyCode){
		case 87:
			document.getElementById("W").style.backgroundColor = "red";
		    break;
		case 83:
			document.getElementById("S").style.backgroundColor = "red";
			break;
		case 65:
			document.getElementById("A").style.backgroundColor = "red";
			break;
		case 68:
			document.getElementById("D").style.backgroundColor = "red";
			break;
	}
});

function getUART(){
	$.get("http://192.168.1.139/uart", function(data){
		var msg = data.split(",");
		console.log(msg);
		if(msg[0] == count + 1){
			count++;
			console.log(msg[1]);
			document.getElementById("message").innerHTML += msg[1];
		}
	});
}

setInterval(getUART,1000)