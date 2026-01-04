function parsePayload(appeui,deveui,payload)
	Tag1 = "dummy"
	strvalue = resiot_hexdecode_ascii(payload)
    
	--Call for LUA Script engine prints
	resiot_debug(string.format("SN_TEST_J Tag: %s Strvalue: %s \n",Tag1, strvalue))
	value = string.match(strvalue, 'Dummy Sensor Value is ([0-9]+)')
	resiot_debug(string.format("SN_TEST_J Tag: %s Value: %s \n",Tag1, value))
	worked, err = resiot_setnodevalue(appeui, deveui, Tag1, value)
    
	if (not worked) then
		resiot_debug(string.format("SN_TEST_J Set Value Error %s \n",err))
	else
		resiot_debug("SN_TEST_J Set Node value successful\n")
	end
end

Origin = resiot_startfrom() --Scene process starts here

if Origin == "Manual" then
	-- Manual script execution for testing
	-- Set your test payload here in hexadecimal
	payload = "44756d6d792053656e736f722056616c756520697320323437"
	-- Set your Application EUI here
	appeui = "70b3d57ed000fc4d"
	-- Set your own Device EUI here
	deveui = "7a39323559379194"
else
	-- Normal execution, get payload received from device
	appeui = resiot_comm_getparam("appeui")
	deveui = resiot_comm_getparam("deveui")
	payload, err = resiot_getlastpayload(appeui, deveui)
 	resiot_debug("SN_TEST_J Test Auto Mode\n")
end

-- Do your stuff
parsePayload(appeui,deveui,payload)