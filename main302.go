// +build example
//
// Do not build by default.

/*
You must have ffmpeg and OpenCV installed in order to run this code. It will connect to the Tello
and then open a window using OpenCV showing the streaming video.
How to run
        go run examples/tello_opencv.go
*/

package main

import (
        "fmt"
        "io"
        "os/exec"
        "strconv"
        "time"
        "gobot.io/x/gobot"
        "gobot.io/x/gobot/platforms/dji/tello"
        "gocv.io/x/gocv"
        "image/color"
        "image"

)

const (
        frameX    = 300
        frameY    = 300
        frameSize = frameX * frameY * 3
        Xcent = frameX/2 
        Ycent = frameY/2
      
)

var epst float64
var detected bool
var posX, posY, moveX, moveY int
var area float64
var flightData *tello.FlightData


func main() {
        
        drone := tello.NewDriver("8890")
        fmt.Println("Drone Connected!")
        window := gocv.NewWindow("Tello")
        window2 := gocv.NewWindow("Color")

        ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
                "-pix_fmt", "bgr24","-r", "20", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "-preset", "ultrafast", "pipe:1")
        ffmpegIn, _ := ffmpeg.StdinPipe()
        ffmpegOut, _ := ffmpeg.StdoutPipe()




        
        work := func() {
                if err := ffmpeg.Start(); err != nil {
                        fmt.Println(err)
                        return
                }
                
                var flightData *tello.FlightData
            
                drone.On(tello.FlightDataEvent, func(data interface{}) {
                        flightData = data.(*tello.FlightData)
                        fmt.Println("Battery:", flightData.BatteryPercentage)
                })

                drone.On(tello.ConnectedEvent, func(data interface{}) {
                        fmt.Println("Connected")
                        drone.StartVideo()
                        drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
                        drone.SetExposure(1)
                        //drone.SetSlowMode()
                        gobot.Every(100*time.Millisecond, func() {
                                drone.StartVideo()
                        })
                })

                drone.On(tello.VideoFrameEvent, func(data interface{}) {
                        pkt := data.([]byte)
                        if _, err := ffmpegIn.Write(pkt); err != nil {
                                fmt.Println(err)
                        }


                })

                
        }

        robot := gobot.NewRobot("tello",
                []gobot.Connection{},
                []gobot.Device{drone},
                work,
        )

        // calling Start(false) lets the Start routine return immediately without an additional blocking goroutine
        robot.Start(false)
        
        epst = 50
        go move(drone)
        go land(drone)
        
        // now handle video frames from ffmpeg stream in main thread, to be macOS/Windows friendly

        for {
                buf := make([]byte, frameSize)
                if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
                        fmt.Println(err)
                        continue
                }
                //reads image
                img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
                if img.Empty() {
                        continue
                }

                //copy of orginal feed
                img2 := gocv.NewMat()
                address := &img
                address.CopyTo(&img2)

                //Blur/Smoothing picture
                //gocv.MedianBlur(img, &img, 5)
                    
                //changes the img from BGR to HSV
                gocv.CvtColor(img,&img,40)
                    
                //HSV lower and upper limits to detect the color blue
                low := gocv.NewScalar(100.0,50.0,50.0,0.0)
                high := gocv.NewScalar(140.0,255.0,255.0,0.0)

                //makes a binary image if the color is in range 
                gocv.InRangeWithScalar(img,low,high,&img)

                //finds the moments of the picture
                moments := gocv.Moments(img,true)
                moment10 := moments["m10"]
                moment01 := moments["m01"]
                area = moments["m00"]
                
                //filter out noise
                if area < 200 {
                    area = 0
                    detected = false
                } else {
                    detected = true
                }

                //center of intensity
                posX = int(moment10/area)
                posY = int(moment01/area)
                //fmt.Println(posX," ",posY)
                //fmt.Println(detected, "\n")

                //draw center
                gocv.Circle(&img2,image.Pt(posX,posY),3,color.RGBA{uint8(255),uint8(100),uint8(100),uint8(100)},2)

                window.IMShow(img)
                window2.IMShow(img2)
                if window.WaitKey(1) >= 0 {
                        drone.Land()
                        break
                }


        }

        
}


func move(d *tello.Driver) {

        d.TakeOff()
        d.Up(40)
        time.Sleep(3000 * time.Millisecond)
        d.Hover()
        time.Sleep(10000 * time.Millisecond)
        //d.Land()

        for{
            moveX = posX - Xcent
            moveY = posY - Ycent

            xarg := float64( (float64(moveX)*epst)/float64(Xcent) )
            yarg := float64( (float64(moveY)*epst)/float64(Ycent) )

            if detected == false {
                d.Hover()
                //d.Clockwise(20)
                //time.Sleep(500 * time.Millisecond)
                fmt.Println("NOT DETECTED")
            } else {
                //x-axis

                d.Right(int(xarg))
                fmt.Print("RIGHT, ")
                fmt.Print(xarg)
                
                //y-axis
                d.Forward(int(yarg))
                fmt.Print("  FORWARD:  ")
                fmt.Println(yarg)
                 
                }




        }

}

func land(d *tello.Driver) {
        //wait 30 sec before intiating landing
        time.Sleep(90000 * time.Millisecond)

        //sets max speed to 20
        epst = 20
        
        //sends 15 down commands every 2 seconds before landing
        i := 0
        for i < 15{
            if area > (0.4*frameX*frameY) {
                d.Land()
                break
            } else{
                d.Down(30)
                time.Sleep(2000 * time.Millisecond)
                fmt.Println("Landing")
                

            }
            i = i + 1
        }
    d.Land()
}






