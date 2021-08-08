<?php

$servername = "localhost";
$dbname = "esp_data";
$username = "set_with_phpMyAdmin_admin";
$password = "set_with_phpMyAdmin_password";

// If you change, the ESP32 sketch needs to match
$api_key_value = "tPmAT5Ab3j7F9";

$api_key = $device = $q0 = $q1 = $q2 = $q3 = $distance = "";

if ($_SERVER["REQUEST_METHOD"] == "POST") 
{
    $api_key = test_input($_POST["api_key"]);

    if($api_key == $api_key_value) 
    {
        $device = test_input($_POST["device"]);
        $q0 = test_input($_POST["q0"]);
        $q1 = test_input($_POST["q1"]);
        $q2 = test_input($_POST["q2"]);
        $q3 = test_input($_POST["q3"]);
	    $distance = test_input($_POST["distance"]);
        
        // Create connection
        $conn = new mysqli($servername, $username, $password, $dbname);

        // Check connection
        if ($conn->connect_error) 
        {
            die("Connection failed: " . $conn->connect_error);
        } 
        
        $sql = "REPLACE INTO SensorData (device, q0, q1, q2, q3, distance)
        VALUES ('" . $device . "', '" . $q0 . "', '" . $q1 . "', '" . $q2 . "', '" . $q3 . "', '" . $distance . "')";
        
        if ($conn->query($sql) === TRUE) 
        {
            echo "New record created successfully";
        } 
        else 
        {
            echo "Error: " . $sql . "<br>" . $conn->error;
        }
    
        $conn->close();
    }
    else 
    {
        echo "Wrong API Key provided.";
    }

}
else 
{
    echo "No data posted with HTTP POST.";
}

function test_input($data) 
{
    $data = trim($data);
    $data = stripslashes($data);
    $data = htmlspecialchars($data);
    return $data;
}

