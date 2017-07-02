using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO.Ports;
using System.Management;
using System.Windows.Forms;
using MazeDrawer.HelperClasses;
using System.Linq;
using System.Threading;


/*
 * TODO list in no particular order
 * 
 * Important stuff:
 * TODO: Determine which robot goes in which direction to completely solve the maze
 * TODO: Communication with robots
 * TODO: Robots in list rather than a switch (See DataReceived(string data)) function 
 * 
 * Extra stuff: 
 * TODO: Add check => If tile is already drawn, skip drawing. It currently redraws a tile if the scanned tile is already drawn. 
 * TODO: Change how the mazes are being compared and merged. It's very inefficient for larger mazes but works for now.
 * TODO: Rename the "DoRobotyStuff" function at some point or another. Also write a better summary for the function. 
 * TODO: Fix the reset button action handler, doesn't work consistently. Maybe function is crap, maybe external factors are causing problems.
 * TODO: Different tabs in application for each robot to draw on (E.g. Main maze, Robot 1, Robot 2, Robot 3, etc...) 
 * TODO: Messages can get lost if multiple robots transmit at the same time (close after one another). Implement a delay either here or on the robots. 
 */

namespace MazeDrawer
{
    public enum Orientation { EAST, SOUTH, WEST, NORTH };
    public enum TileType { STRAIGHT, TTILE, XTILE, CORNER, DEADEND, QUESTION };
    public partial class Form1 : Form
    {
        // Class for reading the Arduino serial input
        private SerialPort serialport;
        private ImageList tiles;
        private Autobot optimus;
        private Autobot bumblebee;
        private List<Autobot> autobots;
        private Graphics graphic;

        private static List<ArrayHelper> mazeList;
        private int xOptimus;
        private int yOptimus;
        private int xBumblebee;
        private int yBumblebee;

        private int centerX;
        private int centerY; 

        private ArrayHelper optimusTile;
        private ArrayHelper bumbleBTile;
        private bool isMerged;

        private int lastMessage;

        private bool messageConfirmed = true;


        public Form1()
        {
            InitializeComponent();
            graphic = CreateGraphics();


            Thread initSerialPort = new Thread(() => InitSerialport());
            initSerialPort.Start();
            
            tiles = new ImageList();
            AddImagesToList();

            mazeList = new List<ArrayHelper>();
            autobots = new List<Autobot>();

            optimus = new Autobot("Optimus", Orientation.NORTH);
            bumblebee = new Autobot("Bumblebee", Orientation.NORTH);

            autobots.Add(optimus);
            autobots.Add(bumblebee);

            xOptimus = 237;
            yOptimus = 287;
            xBumblebee = 737;
            yBumblebee = 287;

            centerX = 475;
            centerY = 275;

            isMerged = false;

            lastMessage = getTimestamp();
        }

        public static List<ArrayHelper> GetMazeList()
        {
            return mazeList;
        }

        /// <summary>
        /// Prepares COM port properties 
        /// </summary>
        private void InitSerialport()
        {
            serialport = new SerialPort();
            serialport.PortName = DetectArduinoPort();
            serialport.BaudRate = 9600;
            serialport.Parity = Parity.None;
            serialport.DataBits = 8;
            serialport.StopBits = StopBits.One;
            serialport.ReceivedBytesThreshold = 1;
            try
            {
                serialport.Open();
            }
            catch(UnauthorizedAccessException e)
            {
                MessageBox.Show("Something went wrong... =(" + Environment.NewLine + "Close whatever is using " + serialport.PortName, "WHOOPS!");
                System.Environment.Exit(1);
            }

            serialport.DataReceived += Serialport_DataReceived;
        }

        /// <summary>
        /// Automatically detects the COM port the Arduino is connected to and returns it 
        /// </summary>
        /// <returns>Returns COM port Arduino is connected to</returns>
        private string DetectArduinoPort()
        {
            ManagementScope connectionScope = new ManagementScope();
            SelectQuery serialQuery = new SelectQuery("SELECT * FROM Win32_SerialPort");
            ManagementObjectSearcher searcher = new ManagementObjectSearcher(connectionScope, serialQuery);

            string message = "Something went wrong... =(" + Environment.NewLine + "Check if the Arduino is connected!";
            string caption = "WHOOPS!";

            try
            {
                foreach(ManagementObject item in searcher.Get())
                {
                    string desc = item["Description"].ToString();
                    string port = item["DeviceID"].ToString();
                    Console.WriteLine(item["Description"].ToString());
                    if (port.Contains("COM5"))
                    {
                        return port;
                    }
                }
            }
            catch(ManagementException e)
            {
                MessageBox.Show(message, caption);
                System.Environment.Exit(1);
            }

            MessageBox.Show(message, caption);
            System.Environment.Exit(1);
            return null;
        }

        /// <summary>
        /// Adds images to the list
        /// </summary>
        private void AddImagesToList()
        {
            tiles.Images.Add(Image.FromFile("..//..//Tiles//deadend.jpg"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//straight.jpg"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//corner.jpg"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//t-tile.jpg"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//x-tile.jpg"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//question.jpg"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//circleOptimus.png"));
            tiles.Images.Add(Image.FromFile("..//..//Tiles//circleBumblebee.png"));
            tiles.ImageSize = new Size(50, 50);
        }

        private void Serialport_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string data = serialport.ReadLine();
            BeginInvoke(new LineReceivedEvent(DataReceived), data);
        }

        private delegate void LineReceivedEvent(string data);

        /// <summary>
        /// Determine the sender
        /// </summary>
        /// <param name="data"></param>
        private void DataReceived(string data)
        {
            char robot = data[0];
            Console.WriteLine(getTimestamp() + " DataReceived: " + data);
            messageConfirmed = true;
            //Console.WriteLine("At: " + unixTimestamp);
            switch (robot)
            {
                case '1':
                    DoRobotyStuff(optimus, data);
                    break;
                case '2':
                    DoRobotyStuff(bumblebee, data);
                    break;
            }

            
        }

        /// <summary>
        /// Do things with robots and tiles 
        /// </summary>
        /// <param name="robot">The robot that sent the data</param>
        /// <param name="data">The data</param>
        private void DoRobotyStuff(Autobot robot, string data)
        {
            switch (data[1])
            {
                case '0':
                    ArrayHelper tileStraight = new ArrayHelper(tiles.Images[1], TileType.STRAIGHT);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileStraight.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            //Orientation is relative to the default position of the tile as depicted in the images
                            tileStraight.TileOrientation = Orientation.EAST;
                            tileStraight.AddDirections(new List<string> { "up", "down" });
                            break;
                        case Orientation.EAST:
                            tileStraight.TileOrientation = Orientation.NORTH;
                            tileStraight.AddDirections(new List<string> { "left", "right" });
                            break;
                        case Orientation.SOUTH:
                            tileStraight.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileStraight.TileOrientation = Orientation.EAST;
                            tileStraight.AddDirections(new List<string> { "up", "down" });
                            break;
                        case Orientation.WEST:
                            tileStraight.TileOrientation = Orientation.NORTH;
                            tileStraight.AddDirections(new List<string> { "left", "right" });
                            break;
                    }
                    UpdateRobotArray(robot, tileStraight, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '1':
                    ArrayHelper tileTLeft = new ArrayHelper(tiles.Images[3], TileType.TTILE);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileTLeft.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileTLeft.TileOrientation = Orientation.EAST;
                            tileTLeft.AddDirections(new List<string> { "up", "left", "down" });
                            break;
                        case Orientation.EAST:
                            tileTLeft.TileImage.RotateFlip(RotateFlipType.Rotate180FlipNone);
                            tileTLeft.TileOrientation = Orientation.SOUTH;
                            tileTLeft.AddDirections(new List<string> { "left", "right", "up" });
                            break;
                        case Orientation.SOUTH:
                            tileTLeft.TileImage.RotateFlip(RotateFlipType.Rotate270FlipNone);
                            tileTLeft.TileOrientation = Orientation.WEST;
                            tileTLeft.AddDirections(new List<string> { "up", "down", "right" });
                            break;
                        case Orientation.WEST:
                            tileTLeft.TileOrientation = Orientation.NORTH;
                            tileTLeft.AddDirections(new List<string> { "left", "right", "down" });
                            break;
                    }
                    UpdateRobotArray(robot, tileTLeft, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '2':
                    ArrayHelper tileTRight = new ArrayHelper(tiles.Images[3], TileType.TTILE);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileTRight.TileImage.RotateFlip(RotateFlipType.Rotate270FlipNone);
                            tileTRight.TileOrientation = Orientation.WEST;
                            tileTRight.AddDirections(new List<string> { "up", "right", "down" });
                            break;
                        case Orientation.EAST:
                            tileTRight.TileOrientation = Orientation.NORTH;
                            tileTRight.AddDirections(new List<string> { "left", "right", "down" });
                            break;
                        case Orientation.SOUTH:
                            tileTRight.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileTRight.TileOrientation = Orientation.EAST;
                            tileTRight.AddDirections(new List<string> { "up", "left", "down" });
                            break;
                        case Orientation.WEST:
                            tileTRight.TileImage.RotateFlip(RotateFlipType.Rotate180FlipNone);
                            tileTRight.TileOrientation = Orientation.SOUTH;
                            tileTRight.AddDirections(new List<string> { "left", "right", "up" });
                            break;
                    }
                    UpdateRobotArray(robot, tileTRight, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '3':
                    ArrayHelper tileX = new ArrayHelper(tiles.Images[4], TileType.XTILE);
                    tileX.TileOrientation = Orientation.NORTH;
                    tileX.AddDirections(new List<string> { "left", "right", "down", "up" });
                    UpdateRobotArray(robot, tileX, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '4':
                    ArrayHelper tileCornerLeft = new ArrayHelper(tiles.Images[2], TileType.CORNER);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileCornerLeft.TileImage.RotateFlip(RotateFlipType.Rotate180FlipNone);
                            tileCornerLeft.TileOrientation = Orientation.SOUTH;
                            tileCornerLeft.AddDirections(new List<string> { "left", "down" });
                            break;
                        case Orientation.EAST:
                            tileCornerLeft.TileImage.RotateFlip(RotateFlipType.Rotate270FlipNone);
                            tileCornerLeft.TileOrientation = Orientation.WEST;
                            tileCornerLeft.AddDirections(new List<string> { "left", "up" });
                            break;
                        case Orientation.SOUTH:
                            tileCornerLeft.TileOrientation = Orientation.NORTH;
                            tileCornerLeft.AddDirections(new List<string> { "up", "right" });
                            break;
                        case Orientation.WEST:
                            tileCornerLeft.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileCornerLeft.TileOrientation = Orientation.EAST;
                            tileCornerLeft.AddDirections(new List<string> { "right", "down" });
                            break;
                    }
                    UpdateRobotArray(robot, tileCornerLeft, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '5':
                    ArrayHelper tileCornerRight = new ArrayHelper(tiles.Images[2], TileType.CORNER);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileCornerRight.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileCornerRight.TileOrientation = Orientation.EAST;
                            tileCornerRight.AddDirections(new List<string> { "right", "down" });
                            break;
                        case Orientation.EAST:
                            tileCornerRight.TileImage.RotateFlip(RotateFlipType.Rotate180FlipNone);
                            tileCornerRight.TileOrientation = Orientation.SOUTH;
                            tileCornerRight.AddDirections(new List<string> { "left", "down" });
                            break;
                        case Orientation.SOUTH:
                            tileCornerRight.TileImage.RotateFlip(RotateFlipType.Rotate270FlipNone);
                            tileCornerRight.TileOrientation = Orientation.WEST;
                            tileCornerRight.AddDirections(new List<string> { "left", "up" });
                            break;
                        case Orientation.WEST:
                            tileCornerRight.TileOrientation = Orientation.NORTH;
                            tileCornerRight.AddDirections(new List<string> { "up", "right" });
                            break;
                    }
                    UpdateRobotArray(robot, tileCornerRight, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '6':
                    ArrayHelper tileT = new ArrayHelper(tiles.Images[3], TileType.TTILE);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileT.TileOrientation = Orientation.NORTH;
                            tileT.AddDirections(new List<string> { "left", "right", "down" });
                            break;
                        case Orientation.EAST:
                            tileT.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileT.TileOrientation = Orientation.EAST;
                            tileT.AddDirections(new List<string> { "left", "up", "down" });
                            break;
                        case Orientation.SOUTH:
                            tileT.TileImage.RotateFlip(RotateFlipType.Rotate180FlipNone);
                            tileT.TileOrientation = Orientation.SOUTH;
                            tileT.AddDirections(new List<string> { "left", "right", "up" });
                            break;
                        case Orientation.WEST:
                            tileT.TileImage.RotateFlip(RotateFlipType.Rotate270FlipNone);
                            tileT.TileOrientation = Orientation.WEST;
                            tileT.AddDirections(new List<string> { "up", "right", "down" });
                            break;
                    }
                    UpdateRobotArray(robot, tileT, data);
                    UpdateRobotOrientation(robot, data);
                    break;
                case '7':
                    ArrayHelper tileDeadEnd = new ArrayHelper(tiles.Images[0], TileType.DEADEND);
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            tileDeadEnd.TileOrientation = Orientation.NORTH;
                            tileDeadEnd.AddDirections(new List<string> { "down" });
                            break;
                        case Orientation.EAST:
                            tileDeadEnd.TileImage.RotateFlip(RotateFlipType.Rotate90FlipNone);
                            tileDeadEnd.TileOrientation = Orientation.EAST;
                            tileDeadEnd.AddDirections(new List<string> { "left" });
                            break;
                        case Orientation.SOUTH:
                            tileDeadEnd.TileImage.RotateFlip(RotateFlipType.Rotate180FlipNone);
                            tileDeadEnd.TileOrientation = Orientation.SOUTH;
                            tileDeadEnd.AddDirections(new List<string> { "up" });
                            break;
                        case Orientation.WEST:
                            tileDeadEnd.TileImage.RotateFlip(RotateFlipType.Rotate270FlipNone);
                            tileDeadEnd.TileOrientation = Orientation.WEST;
                            tileDeadEnd.AddDirections(new List<string> { "right" });
                            break;
                    }
                    UpdateRobotArray(robot, tileDeadEnd, data);
                    UpdateRobotOrientation(robot, data);
                    break;
            }
        }

        /// <summary>
        /// Adjusts a robot's orientation.
        /// </summary>
        /// <param name="robot">The Robot</param>
        /// <param name="data">The data</param>
        private void UpdateRobotOrientation(Autobot robot, string data)
        {
            switch (data[2])
            {
                case '1':
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            robot.Orientation = Orientation.EAST;
                            break;
                        case Orientation.EAST:
                            robot.Orientation = Orientation.SOUTH;
                            break;
                        case Orientation.SOUTH:
                            robot.Orientation = Orientation.WEST;
                            break;
                        case Orientation.WEST:
                            robot.Orientation = Orientation.NORTH;
                            break;
                    }
                    break;
                case '2':
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            robot.Orientation = Orientation.SOUTH;
                            break;
                        case Orientation.EAST:
                            robot.Orientation = Orientation.WEST;
                            break;
                        case Orientation.SOUTH:
                            robot.Orientation = Orientation.NORTH;
                            break;
                        case Orientation.WEST:
                            robot.Orientation = Orientation.EAST;
                            break;
                    }
                    break;
                case '3':
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            robot.Orientation = Orientation.WEST;
                            break;
                        case Orientation.EAST:
                            robot.Orientation = Orientation.NORTH;
                            break;
                        case Orientation.SOUTH:
                            robot.Orientation = Orientation.EAST;
                            break;
                        case Orientation.WEST:
                            robot.Orientation = Orientation.SOUTH;
                            break;
                    }
                    break;
                default:
                    break;
            }
        }

        /// <summary>
        /// Adds tile to the robot-specific array
        /// </summary>
        /// <param name="robot">The robot</param>
        /// <param name="tile">The tile</param>
        private void UpdateRobotArray(Autobot robot, ArrayHelper tile, string data)
        {

            if (robot.TileArray.Count > 0)
            {
                switch (robot.Orientation)
                {
                    case Orientation.NORTH:
                        robot.Y--;
                        break;
                    case Orientation.EAST:
                        robot.X++;
                        break;
                    case Orientation.SOUTH:
                        robot.Y++;
                        break;
                    case Orientation.WEST:
                        robot.X--;
                        break;
                }
            }

            tile.X = robot.X;
            tile.Y = robot.Y;

            if (isMerged)
            {
                //set deltaX and deltaY
                switch (robot.Name)
                {
                    case "Optimus":
                        tile.DeltaX = tile.X - optimusTile.DeltaX;
                        tile.DeltaY = tile.Y - optimusTile.DeltaY;
                        break;
                    case "Bumblebee":
                        tile.DeltaX = tile.X - bumbleBTile.DeltaX;
                        tile.DeltaY = tile.Y - bumbleBTile.DeltaY;
                        break;
                }
            }

            robot.AddToArray(tile);
            DrawTile(robot, tile);

            if (!isMerged)
            {
                CompareMaze(robot);
                if (!isMerged)
                {
                    addMessageToSend("09", robot);
                }
            }
        }

        /// <summary>
        /// Draws the tiles
        /// </summary>
        private void DrawTile(Autobot robot, ArrayHelper helper)
        {
            int helperOffsetX = helper.X * 50;
            int helperOffsetY = helper.Y * 50;

            if (!isMerged)
            {
                switch (robot.Name)
                {
                    case "Optimus":
                        graphic.DrawImage(helper.TileImage, xOptimus + helperOffsetX, yOptimus + helperOffsetY);
                        break;
                    case "Bumblebee":
                        graphic.DrawImage(helper.TileImage, xBumblebee + helperOffsetX, yBumblebee + helperOffsetY);
                        break;
                }
            }
            else
            {
                graphic.DrawImage(helper.TileImage, centerX + helperOffsetX, centerY + helperOffsetY);

                switch (robot.Name)
                {
                    case "Optimus":
                        graphic.DrawImage(tiles.Images[6], centerX + helperOffsetX, centerY + helperOffsetY);
                        break;
                    case "Bumblebee":
                        graphic.DrawImage(tiles.Images[7], centerX + helperOffsetX, centerY + helperOffsetY);
                        break;
                }

                ArrayHelper prev = robot.TileArray[robot.TileArray.Count - 2];
                graphic.DrawImage(prev.TileImage, centerX + prev.DeltaX * 50, centerY + prev.DeltaY * 50);

                ArrayHelper temp = mazeList.Where(t => t.DeltaX == helper.DeltaX && t.DeltaY == helper.DeltaY).FirstOrDefault();

                if (temp == null)
                {
                    mazeList.Add(helper);
                }

                FindLooseEnds();
            }
        }

        /// <summary>
        /// Draws merged maze
        /// </summary>
        /// <param name="list">List of tiles</param>
        private void DrawMaze(List<ArrayHelper> list)
        {
            // Clear current mazes first
            graphic.Clear(Color.White);

            foreach(ArrayHelper tile in list)
            {
                graphic.DrawImage(tile.TileImage, centerX + tile.DeltaX * 50, centerY - tile.DeltaY * -50);
            }

            foreach(Autobot robot in autobots)
            {
                if (robot.Name.Equals("Optimus"))
                {
                    graphic.DrawImage(tiles.Images[6], centerX + robot.X * 50, centerY - robot.Y * -50);
                }
                else if (robot.Name.Equals("Bumblebee"))
                {
                    graphic.DrawImage(tiles.Images[7], centerX + robot.X * 50, centerY - robot.Y * -50);
                }
            }
        }

        /// <summary>
        /// Compares the maze of both robots to check for overlap
        /// </summary>
        private void CompareMaze(Autobot robot)
        {
            if(optimus.TileArray.Count != 0)
            {
                optimusTile = optimus.TileArray.Last();
                ArrayHelper optimusEast = (ArrayHelper)optimus.TileArray.Where(o => o.X == optimusTile.X + 1 && o.Y == optimusTile.Y).FirstOrDefault();
                ArrayHelper optimusSouth = (ArrayHelper)optimus.TileArray.Where(o => o.Y == optimusTile.Y + 1 && o.X == optimusTile.X).FirstOrDefault();
                ArrayHelper optimusWest = (ArrayHelper)optimus.TileArray.Where(o => o.X == optimusTile.X - 1 && o.Y == optimusTile.Y).FirstOrDefault();
                ArrayHelper optimusNorth = (ArrayHelper)optimus.TileArray.Where(o => o.Y == optimusTile.Y - 1 && o.X == optimusTile.X).FirstOrDefault();
                int counter = 0;

                // Arbitrary amount of tiles that need to have been scanned before the computer starts comparing the two mazes 
                if (optimus.TileArray.Count >= 5 && bumblebee.TileArray.Count >= 5)
                {
                    foreach (ArrayHelper bumbleB in bumblebee.TileArray.Where(b => b.TileType.Equals(optimusTile.TileType) && b.TileOrientation.Equals(optimusTile.TileOrientation)))
                    {
                        ArrayHelper bumblebeeEast = (ArrayHelper)bumblebee.TileArray.Where(b => b.X == bumbleB.X + 1 && b.Y == bumbleB.Y).FirstOrDefault();
                        ArrayHelper bumblebeeSouth = (ArrayHelper)bumblebee.TileArray.Where(b => b.Y == bumbleB.Y + 1 && b.X == bumbleB.X).FirstOrDefault();
                        ArrayHelper bumblebeeWest = (ArrayHelper)bumblebee.TileArray.Where(b => b.X == bumbleB.X - 1 && b.Y == bumbleB.Y).FirstOrDefault();
                        ArrayHelper bumblebeeNorth = (ArrayHelper)bumblebee.TileArray.Where(b => b.Y == bumbleB.Y - 1 && b.X == bumbleB.X).FirstOrDefault();

                        // Is ugly, will remain ugly until an alternative is found
                        if (optimusEast != null && bumblebeeEast != null)
                        {
                            if (optimusEast.TileType.Equals(bumblebeeEast.TileType) && optimusEast.TileOrientation.Equals(bumblebeeEast.TileOrientation))
                            {
                                counter++;
                            }
                        }

                        if (optimusSouth != null && bumblebeeSouth != null)
                        {
                            if (optimusSouth.TileType.Equals(bumblebeeSouth.TileType) && optimusSouth.TileOrientation.Equals(bumblebeeSouth.TileOrientation))
                            {
                                counter++;
                            }
                        }


                        if (optimusWest != null && bumblebeeWest != null)
                        {
                            if (optimusWest.TileType.Equals(bumblebeeWest.TileType) && optimusWest.TileOrientation.Equals(bumblebeeWest.TileOrientation))
                            {
                                counter++;
                            }
                        }

                        if (optimusNorth != null && bumblebeeNorth != null)
                        {
                            if (optimusNorth.TileType.Equals(bumblebeeNorth.TileType) && optimusNorth.TileOrientation.Equals(bumblebeeNorth.TileOrientation))
                            {
                                counter++;
                            }
                        }

                        if (counter >= 3)
                        {
                            // If 3 or more tiles match, go ahead and merge the two mazes.
                            bumbleBTile = bumbleB;
                            MergeMaze();
                            break;
                        }
                        else
                        {
                            counter = 0;
                        }
                    }
                }
            }
        }
        
        /// <summary>
        /// "Fixes" the robots' coordinates after merging the mazes
        /// </summary>
        /// <param name="bumbleB"></param>
        /// <param name="optimus"></param>
        private void FixRobotCoordinates(ArrayHelper bumbleB, ArrayHelper optimus)
        {
            this.optimus.X = this.optimus.X - optimus.X;
            this.optimus.Y = this.optimus.Y - optimus.Y;
            bumblebee.X = bumblebee.X - bumbleB.X;
            bumblebee.Y = bumblebee.Y - bumbleB.Y;
        }

        /// <summary>
        /// Merges both mazes
        /// </summary>
        /// <param name="bumbleB">Bumblebee tile</param>
        /// <param name="optimus">Optimus tile</param>
        private void MergeMaze()
        {
            FixRobotCoordinates(bumbleBTile, optimusTile);

            foreach(ArrayHelper tile in bumblebee.TileArray)
            {
                tile.DeltaX = tile.X - bumbleBTile.X;
                tile.DeltaY = tile.Y - bumbleBTile.Y;
                ArrayHelper temp = mazeList.Where(t => t.DeltaX == tile.DeltaX && t.DeltaY == tile.DeltaY).FirstOrDefault();

                if(temp == null)
                {
                    mazeList.Add(tile);
                }
            }
            foreach(ArrayHelper tile in this.optimus.TileArray)
            {
                tile.DeltaX = tile.X - optimusTile.X;
                tile.DeltaY = tile.Y - optimusTile.Y;
                ArrayHelper temp = mazeList.Where(t => t.DeltaX == tile.DeltaX && t.DeltaY == tile.DeltaY).FirstOrDefault();

                if(temp == null)
                {
                    mazeList.Add(tile);
                }
            }
            isMerged = true;
            DrawMaze(mazeList);

            FindLooseEnds();
        }

        /// <summary>
        /// Find open ends and slap down "?" tiles 
        /// </summary>
        private void FindLooseEnds()
        {
            List<ArrayHelper> questionmarks = new List<ArrayHelper>();

            foreach(ArrayHelper helper in mazeList)
            {
                int x = helper.DeltaX;
                int y = helper.DeltaY;

                foreach(string direction in helper.Directions)
                {
                    switch (direction)
                    {
                        case "up":
                            ArrayHelper upTile = mazeList.Where(u => u.DeltaX == helper.DeltaX && u.DeltaY == helper.DeltaY - 1).FirstOrDefault();
                            if(upTile == null)
                            {
                                // draw "?" tile above helper
                                graphic.DrawImage(tiles.Images[5], centerX + x * 50, centerY - 50 + y * 50);
                                ArrayHelper qTile = new ArrayHelper(tiles.Images[5], TileType.QUESTION, x, y - 1);
                                ArrayHelper temp = questionmarks.Where(t => t.DeltaX == qTile.DeltaX && t.DeltaY == qTile.DeltaY).FirstOrDefault();
                                if (temp == null)
                                {
                                    questionmarks.Add(qTile);
                                }
                            }
                            break;
                        case "down":
                            ArrayHelper downTile = mazeList.Where(d => d.DeltaX == helper.DeltaX && d.DeltaY == helper.DeltaY + 1).FirstOrDefault();
                            if(downTile == null)
                            {
                                // draw "?" tile below helper
                                graphic.DrawImage(tiles.Images[5], centerX + x * 50, centerY + 50 + y * 50);
                                ArrayHelper qTile = new ArrayHelper(tiles.Images[5], TileType.QUESTION, x, y + 1);
                                ArrayHelper temp = questionmarks.Where(t => t.DeltaX == qTile.DeltaX && t.DeltaY == qTile.DeltaY).FirstOrDefault();
                                if (temp == null)
                                {
                                    questionmarks.Add(qTile);
                                }
                            }
                            break;
                        case "left":
                            ArrayHelper leftTile = mazeList.Where(l => l.DeltaX == helper.DeltaX - 1 && l.DeltaY == helper.DeltaY).FirstOrDefault();
                            if (leftTile == null)
                            {
                                // draw "?" tile to the left helper
                                graphic.DrawImage(tiles.Images[5], centerX - 50 + x * 50, centerY + y * 50);
                                ArrayHelper qTile = new ArrayHelper(tiles.Images[5], TileType.QUESTION, x - 1, y);
                                ArrayHelper temp = questionmarks.Where(t => t.DeltaX == qTile.DeltaX && t.DeltaY == qTile.DeltaY).FirstOrDefault();
                                if (temp == null)
                                {
                                    questionmarks.Add(qTile);
                                }
                            }
                            break;
                        case "right":
                            ArrayHelper rightTile = mazeList.Where(r => r.DeltaX == helper.DeltaX + 1 && r.DeltaY == helper.DeltaY).FirstOrDefault();
                            if (rightTile == null)
                            {
                                // draw "?" tile to the left helper
                                graphic.DrawImage(tiles.Images[5], centerX + 50 + x * 50, centerY + y * 50);
                                ArrayHelper qTile = new ArrayHelper(tiles.Images[5], TileType.QUESTION, x + 1, y);
                                ArrayHelper temp = questionmarks.Where(t => t.DeltaX == qTile.DeltaX && t.DeltaY == qTile.DeltaY).FirstOrDefault();
                                if (temp == null)
                                {
                                    questionmarks.Add(qTile);
                                }
                            }
                            break;
                    }
                }
            }
            findPath(questionmarks);
        }

        private void findPath(List<ArrayHelper> questionmarks)
        {
            messagebox.Items.Clear();
            Astar astar = new Astar();

            foreach (Autobot autobot in autobots)
            {
                List<ArrayHelper> path = new List<ArrayHelper>();
                ArrayHelper start = (ArrayHelper)mazeList.Where(t => t.DeltaX == autobot.X && t.DeltaY == autobot.Y).FirstOrDefault();

                List<ArrayHelper> shortestPath = new List<ArrayHelper>();

                foreach (ArrayHelper questionmark in questionmarks)
                {
                    path = astar.FindPath(start, questionmark);

                    if (path.Count < shortestPath.Count || shortestPath.Count == 0)
                    {
                        shortestPath = path;
                        messagebox.Items.Add("New shortest path");
                    }

                    foreach (ArrayHelper helper in mazeList.Where(h => h.State != ArrayHelper.HelperState.UNTESTED))
                    {
                        helper.State = ArrayHelper.HelperState.UNTESTED;
                        helper.ParentArrayHelper = null;
                    }

                    messagebox.Items.Add("Path " + autobot.Name + " to " + questionmark.DeltaX + " " + questionmark.DeltaY);
                    foreach (ArrayHelper helper in path)
                    {
                        messagebox.Items.Add(helper.DeltaX + " " + helper.DeltaY);
                    }
                }

                Console.WriteLine("BOT: " + autobot.Name);
                Console.WriteLine("POS: " + autobot.X + " " + autobot.Y);
                Console.WriteLine("TO:  " + shortestPath[0].DeltaX + " " + shortestPath[0].DeltaY);
                Console.WriteLine("DIR:  " + autobot.Orientation);

                var dirX = shortestPath[0].DeltaX - autobot.X;
                var dirY = shortestPath[0].DeltaY - autobot.Y;

                Orientation newOrientation = new Orientation();
                if(dirX > 0)
                {
                    newOrientation = Orientation.EAST;
                }
                else if(dirX < 0)
                {
                    newOrientation = Orientation.WEST;
                }
                else if(dirY > 0)
                {
                    newOrientation = Orientation.SOUTH;
                }
                else if (dirY < 0)
                {
                    newOrientation = Orientation.NORTH;
                }
                Console.WriteLine("NEW:  " + newOrientation);
                int turnDirection = getTurnDirection(newOrientation, autobot);
                Console.WriteLine("NEW:  " + turnDirection);
                addMessageToSend("0" + turnDirection, autobot);
            }
        }

        private void addMessageToSend(string data, Autobot robot)
        {
            Thread t = new Thread(() => SendData(data, robot));
            t.Start();

        }

        /// <summary>
        /// Tell robot which way to go
        /// </summary>
        private void SendData(string data, Autobot robot)
        {


            string message;
            int prevTs = 0;
            //getTimestamp() - lastMessage < 5
            while (!messageConfirmed)
            {
                // wait
                if (getTimestamp() != prevTs)
                {
                    Console.WriteLine(getTimestamp() + " Waiting..");
                    prevTs = getTimestamp();
                }
            }

            lastMessage = getTimestamp();
            messageConfirmed = false;

            switch (robot.Name)
            {
                case "Optimus":
                    message = "1" + data;
                    serialport.Write(message);
                    Console.WriteLine(lastMessage + " DataSend: " + message);
                    break;
                case "Bumblebee":
                    message = "2" + data;
                    serialport.Write(message);
                    Console.WriteLine(lastMessage + " DataSend: " + message);
                    break;
            }
        }

        private int getTurnDirection(Orientation targetOrientation, Autobot robot)
        {
            int direction = 0;

            switch (targetOrientation)
            {
                case Orientation.NORTH:
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            direction = 0;
                            break;
                        case Orientation.EAST:
                            direction = 1;
                            break;
                        case Orientation.SOUTH:
                            direction = 2;
                            break;
                        case Orientation.WEST:
                            direction = 3;
                            break;
                    }
                    break;
                case Orientation.EAST:
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            direction = 3;
                            break;
                        case Orientation.EAST:
                            direction = 0;
                            break;
                        case Orientation.SOUTH:
                            direction = 1;
                            break;
                        case Orientation.WEST:
                            direction = 2;
                            break;
                    }
                    break;
                case Orientation.SOUTH:
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            direction = 2;
                            break;
                        case Orientation.EAST:
                            direction = 3;
                            break;
                        case Orientation.SOUTH:
                            direction = 0;
                            break;
                        case Orientation.WEST:
                            direction = 1;
                            break;
                    }
                    break;
                case Orientation.WEST:
                    switch (robot.Orientation)
                    {
                        case Orientation.NORTH:
                            direction = 1;
                            break;
                        case Orientation.EAST:
                            direction = 2;
                            break;
                        case Orientation.SOUTH:
                            direction = 3;
                            break;
                        case Orientation.WEST:
                            direction = 0;
                            break;
                    }
                    break;
            }

            return direction;

        }

        /// <summary>
        /// Resets the application, sort of, doesn't really work yet. 
        /// </summary>
        /// <param name="sender">Sender</param>
        /// <param name="e">Event arguments</param>
        private void resetMazeBtn_Click(object sender, EventArgs e)
        {
            xOptimus = 237;
            yOptimus = 287;
            xBumblebee = 737;
            yBumblebee = 287;
            optimus.TileArray.Clear();
            bumblebee.TileArray.Clear();
            optimus.Orientation = Orientation.NORTH;
            bumblebee.Orientation = Orientation.NORTH;
            graphic.Clear(Color.White);
        }

        /// <summary>
        /// Test button to test the data transmission to the listener
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            //Console.WriteLine(getTimestamp() + " Charge");

            addMessageToSend("09", bumblebee);

            //SendData("35", optimus);
        }

        private int getTimestamp()
        {
            return (int)(DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1))).TotalSeconds;
        }
    }
}