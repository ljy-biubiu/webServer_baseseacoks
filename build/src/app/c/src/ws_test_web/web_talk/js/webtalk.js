var ws;
var jsonObj;
var flag_test = 0;

$(function () {
    ws = new WebSocket('ws://' + document.location.host + '/ws');

    ws.onopen = function () {
        console.log('onopen');
    };
    ws.onclose = function () {
        $('#message').text('Lost connection.');
        console.log('onclose');
    };
    ws.onmessage = function (message) {
        //console.log("got '" + message.data + "'");
        eval(message.data);
    };
    ws.onerror = function (error) {
        console.log('onerror ' + error);
        console.log(error);
    };
    $('#count').click(function () {
        ws.send($('#count').val());
    });
    $('#close').click(function () {
        ws.send('close');
    });
    $('#die').click(function () {
        ws.send('die');
    });
    $('#load').click(function () {
        ws.send('load');
    });
});

pointcloud = function (value) {
    //console.log(pointcloud);
    flag_test =1 ;
    jsonObj = window.JSON.parse(value);
    console.log("jsonObj");
}

set = function (value) {
    $('#count').val(value)
}
