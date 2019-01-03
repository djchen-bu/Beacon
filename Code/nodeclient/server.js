var express = require('express');
var app = require('express')();
var http = require('http').Server(app);

app.use("/", express.static(__dirname + '/'));

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});