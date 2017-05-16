using System.Collections.Generic;
using MazeDrawer.HelperClasses;

namespace MazeDrawer
{
    class Autobot
    {
        private string name;
        private List<ArrayHelper> tileArray;
        private int x;
        private int y;
        private Orientation orientation;

        public Autobot(string name, Orientation orientation)
        {
            this.name = name;
            this.orientation = orientation;
            this.tileArray = new List<ArrayHelper>();
            this.x = 0;
            this.y = 0;

        }

        public string Name { get { return this.name; } set { this.name = value; } }
        public List<ArrayHelper> TileArray { get { return this.tileArray; } set { this.tileArray = value; } }
        public Orientation Orientation { get { return this.orientation; } set { this.orientation = value; } }
        public int X { get { return this.x; } set { this.x = value; } }
        public int Y { get { return this.y; } set { this.y = value; } }
        
        /// <summary>
        /// Adds ArrayHelper object to the array
        /// </summary>
        /// <param name="obj"></param>
        public void AddToArray(ArrayHelper obj)
        {
            tileArray.Add(obj);
        }
    }
}