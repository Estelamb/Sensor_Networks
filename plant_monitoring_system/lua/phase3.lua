-- Helper function to convert bytes to Integer (Little Endian)
function bytesToInt(bytes, start, size, signed)
    local val = 0
    for i = 0, size - 1 do
        val = val + (bytes[start + i] * (256 ^ i))
    end
    if signed and val >= (256 ^ size) / 2 then
        val = val - (256 ^ size)
    end
    return val
end

function parsePayload(appeui, deveui, payload)
    -- Convert hex payload to byte array
    local bytes = resiot_hexdecode(payload)

    -- 1. GPS Data (1 to 17)
    local lat  = bytesToInt(bytes, 1, 4, true) / 1000000.0
    local lon  = bytesToInt(bytes, 5, 4, true) / 1000000.0
    local alt  = bytesToInt(bytes, 9, 4, true) / 100.0 
    
    -- GPS Time (13-15) HHMMSS
    local hh = bytes[13]
    local mm = bytes[14]
    local ss = bytes[15]
    local time = string.format("%02d:%02d:%02d", hh, mm, ss)

    local sats = bytes[16]                            

    -- 2. Temperature and Humidity Data (17 to 20)
    local temp = bytesToInt(bytes, 17, 2, true) / 100.0
    local hum  = bytesToInt(bytes, 19, 2, false) / 100.0

    -- 3. Brightness and Moisture (21 to 24)
    local light    = bytesToInt(bytes, 21, 2, false) / 10.0
    local moisture = bytesToInt(bytes, 23, 2, false) / 10.0

    -- 4. Color RGB (Offsets 25 to 27)
    local r = bytes[25]
    local g = bytes[26]
    local b = bytes[27]

    -- 5. Accelerometer (28 to 30)
    -- These are int8 (signed). If value > 127, it's negative.
    local function toInt8(b) return b > 127 and b - 256 or b end
    local x = toInt8(bytes[28]) / 10.0
    local y = toInt8(bytes[29]) / 10.0
    local z = toInt8(bytes[30]) / 10.0

    -- Debug Logs
    resiot_debug(string.format("GPS: Lat: %.6f, Long: %.6f, Alt: %.2f, Time: %s, Sats: %d", lat, lon, alt, time, sats))
    resiot_debug(string.format("Sensors: Temp: %.2f, Hum: %.2f, Light: %.1f, Moisture: %.1f", temp, hum, light, moisture))
    resiot_debug(string.format("Color: R:%d, G:%d, B:%d", r, g, b))
    resiot_debug(string.format("Accel: X:%.1f, Y:%.1f, Z:%.1f", x, y, z))

    -- Update Nodes in ResIoT
    resiot_setnodevalue(appeui, deveui, "Latitude", lat)
    resiot_setnodevalue(appeui, deveui, "Longitude", lon)
    resiot_setnodevalue(appeui, deveui, "Altitude", alt)
    resiot_setnodevalue(appeui, deveui, "Time", time)
    resiot_setnodevalue(appeui, deveui, "Sats", sats)
    resiot_setnodevalue(appeui, deveui, "Temperature", temp)
    resiot_setnodevalue(appeui, deveui, "Humidity", hum)
    resiot_setnodevalue(appeui, deveui, "Light", light)
    resiot_setnodevalue(appeui, deveui, "Moisture", moisture)
    resiot_setnodevalue(appeui, deveui, "Red", r)
    resiot_setnodevalue(appeui, deveui, "Green", g)
    resiot_setnodevalue(appeui, deveui, "Blue", b)
    resiot_setnodevalue(appeui, deveui, "X", x)
    resiot_setnodevalue(appeui, deveui, "Y", y)
    resiot_setnodevalue(appeui, deveui, "Z", z)
end

-- --- Main Process ---
Origin = resiot_startfrom()

if Origin == "Manual" then
    -- Test payload (30 bytes hex)
    payload = "010203040506070809101112131415161718192021222324252627282930" 
    appeui = "70b3d57ed000fc4d"
    deveui = "7a39323559379194"
else
    appeui = resiot_comm_getparam("appeui")
    deveui = resiot_comm_getparam("deveui")
    payload, err = resiot_getlastpayload(appeui, deveui)
end

parsePayload(appeui, deveui, payload)