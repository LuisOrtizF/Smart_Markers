<!DOCTYPE html>

<!-- Refresh the webpage every 1 seconds -->
<?php
	$url=$_SERVER['REQUEST_URI'];
	header("Refresh: 1; URL=$url");  
?>

<html>
        <center>
        <h1 style="color:black;font-size:40px;">Smart Square Fiducial Markers</h1>

        <body>
                <?php
                        $servername = "localhost";
                        $dbname = "esp_data";
                        $username = "set_with_phpMyAdmin_admin";
                        $password = "set_with_phpMyAdmin_password";

                        // Create connection
                        $conn = new mysqli($servername, $username, $password, $dbname);

                        // Check connection
                        if ($conn->connect_error) {
                        die("Connection failed: " . $conn->connect_error);
                        } 

                        $sql = "SELECT device, q0, q1, q2, q3, distance, reading_time FROM SensorData ORDER BY cast(device as unsigned) ASC";

                        echo '<hr />';

                        echo '<table cellspacing="5" cellpadding="5">
                        <tr> 
                                <td><b>Marker ID</b></td> 
                                <td><b>Quaternion [w]</b></td> 
                                <td><b>Quaternion [x]</b></td> 
                                <td><b>Quaternion [y]</b></td>
                                <td><b>Quaternion [z]</b></td>
                                <td><b>Distance [m]</b></td>  
                                <td><b>Timestamp [us]</b></td> 
                        </tr>';
                        
                        if ($result = $conn->query($sql)) {

                                while ($row = $result->fetch_assoc()) {
                                        
                                        $row_device = $row["device"];
                                        $row_q0 = $row["q0"];
                                        $row_q1 = $row["q1"];
                                        $row_q2 = $row["q2"]; 
                                        $row_q3 = $row["q3"];
                                        $row_distance = $row["distance"];  
                                        $row_reading_time = microtime(true);

                                        echo '<tr> 
                                                <td>' . $row_device . '</td> 
                                                <td>' . $row_q0 . '</td> 
                                                <td>' . $row_q1 . '</td> 
                                                <td>' . $row_q2 . '</td>
                                                <td>' . $row_q3 . '</td> 
                                                <td>' . $row_distance . '</td> 
                                                <td>' . $row_reading_time . '</td> 
                                        </tr>';
                                        
                                }

                                $result->free();
                        }

                        // Exporting mysql table to .txt file using PHP button

                        $list = array("0", "0");

                        if(isset($_POST['button1'])) { 

                                $file = "pms.txt";
                                chmod($file, 0777); 
                                $fp = fopen( $file ,"w") or die("File does not exist!");
                                
                                if ($rs = $conn->query("SELECT distance, q1, q2, q3, q0, device FROM SensorData ORDER BY cast(device as unsigned) ASC"))
                                {
                                        while ($row = $rs->fetch_assoc())
                                        {
                                                fputcsv($fp, array_merge( $list, $row), " ");
                                        }
                                        $rs->close();
                                }

                                fclose($fp);
                        } 

                        // Clear mysql table using PHP button

                        if(isset($_POST['button2']) ) { 
                                $sql = "DELETE FROM SensorData";

                                if ($conn->query($sql) === TRUE){
                                        echo "";
                                }
                                else{
                                        echo "" . $conn->error;
                                }
                        }

                        $conn->close();
                        
                ?>

                </table> 

                <?php echo '<hr />'; // line bottom?> 

                <!-- SAVE AND DELETE buttons -->

                <form method="post"> 
                        <input type="submit" name="button1"
                                value="Save"/> 

                        <?php echo'&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp';?> 

                        <input type="submit" name="button2"
                                value="Delete"/>
                </form> 

                <!-- UFRN LOGO -->

                <img src="ufrn_logo.png" width="200" height="121" align="right">

        </body>

</html>
