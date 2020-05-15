
function Decode(fPort, bytes) {
    var ec = (bytes[0] << 8) | bytes[1];
    var ph = (bytes[2] << 8) | bytes[3];

    var dataout = {
        "sensorvalues": {
            'ec': ec / 100,
            'ph': ph / 100
        },
    };
    return dataout;
}