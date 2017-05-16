using System;
using System.Collections.Generic;
using System.Linq;

namespace MazeDrawer.HelperClasses
{
    class Astar
    {
        List<ArrayHelper> mazeList;

        public Astar()
        {
            mazeList = Form1.GetMazeList();
        }

        private List<ArrayHelper> FindSurroundingTiles(ArrayHelper tile, ArrayHelper end)
        {
            List<ArrayHelper> surroundingTiles = new List<ArrayHelper>();

            
            foreach(string direction in tile.Directions)
            {
                switch (direction)
                {
                    case "up":
                        ArrayHelper upTile = mazeList.Where(u => u.DeltaX == tile.DeltaX && u.DeltaY == tile.DeltaY - 1).FirstOrDefault();
                        if(upTile != null)
                        {
                            if(upTile.TileType != TileType.DEADEND)
                            {
                                surroundingTiles.Add(upTile);
                            }
                        }
                        else if (tile.DeltaX == end.DeltaX && tile.DeltaY - 1 == end.DeltaY)
                        {
                            surroundingTiles.Add(end);
                        }
                        else
                        {
                            break;
                        }
                        break;
                    case "down":
                        ArrayHelper downTile = mazeList.Where(d => d.DeltaX == tile.DeltaX && d.DeltaY == tile.DeltaY + 1).FirstOrDefault();
                        if(downTile != null)
                        {
                            if (downTile.TileType != TileType.DEADEND)
                            {
                                surroundingTiles.Add(downTile);
                            }
                        }
                        else if (tile.DeltaX == end.DeltaX && tile.DeltaY + 1 == end.DeltaY)
                        {
                            surroundingTiles.Add(end);
                        }
                        else
                        {
                            break;
                        }
                        break;
                    case "left":
                        ArrayHelper leftTile = mazeList.Where(l => l.DeltaX == tile.DeltaX - 1 && l.DeltaY == tile.DeltaY).FirstOrDefault();
                        if(leftTile != null)
                        {
                            if (leftTile.TileType != TileType.DEADEND)
                            {
                                surroundingTiles.Add(leftTile);
                            }
                        }
                        else if (tile.DeltaX - 1 == end.DeltaX && tile.DeltaY == end.DeltaY)
                        {
                            surroundingTiles.Add(end);
                        }
                        else
                        {
                            break;
                        }
                        break;
                    case "right":
                        ArrayHelper rightTile = mazeList.Where(r => r.DeltaX == tile.DeltaX + 1 && r.DeltaY == tile.DeltaY).FirstOrDefault();
                        if(rightTile != null)
                        {
                            if (rightTile.TileType != TileType.DEADEND)
                            {
                                surroundingTiles.Add(rightTile);
                            }
                        }
                        else if (tile.DeltaX + 1 == end.DeltaX && tile.DeltaY == end.DeltaY)
                        {
                            surroundingTiles.Add(end);
                        }
                        else
                        {
                            break;
                        }
                        break;
                }
            }
            return surroundingTiles;
        }

        private void setValues(List<ArrayHelper> surroundingTiles, ArrayHelper start, ArrayHelper end)
        {
            int x, y;
            foreach(ArrayHelper tile in surroundingTiles)
            {
                if(tile.DeltaX > end.DeltaX && (tile.DeltaX >= 0 || end.DeltaX >= 0))
                {
                    x = Math.Abs(tile.DeltaX - Math.Abs(end.DeltaX));
                }
                else
                {
                    x = Math.Abs(end.DeltaX) - Math.Abs(tile.DeltaX);
                }

                if (tile.DeltaY > end.DeltaY && (tile.DeltaY >= 0 || end.DeltaY >= 0))
                {
                    y = Math.Abs(tile.DeltaY - Math.Abs(end.DeltaY));
                }
                else
                {
                    y = Math.Abs(end.DeltaY) - Math.Abs(tile.DeltaY);
                }
                
                tile.H = x + y;
                tile.G = start.G + 1;
            }
        }

        private List<ArrayHelper> GetAdjacentTiles(ArrayHelper startTile, ArrayHelper endTile)
        {
            List<ArrayHelper> options = new List<ArrayHelper>();
            List<ArrayHelper> surroundingTiles = FindSurroundingTiles(startTile, endTile);
            setValues(surroundingTiles, startTile, endTile);

            foreach (ArrayHelper tile in surroundingTiles)
            {
                // Ignore already-closed tiles
                if (tile.State == ArrayHelper.HelperState.CLOSED)
                    continue;

                // Already-open tiles are only added to the list if their G-value is lower going via this route.
                if (tile.State == ArrayHelper.HelperState.OPEN)
                {
                    if (startTile.G++ < tile.G)
                    {
                        tile.ParentArrayHelper = startTile;
                        options.Add(tile);
                    }
                }
                else
                {
                    // If it's untested, set the parent and flag it as 'Open' for consideration
                    tile.ParentArrayHelper = startTile;
                    tile.State = ArrayHelper.HelperState.OPEN;
                    options.Add(tile);
                }
            }
            return options;
        }

        private bool Search(ArrayHelper helper, ArrayHelper end)
        {
            helper.State = ArrayHelper.HelperState.CLOSED;
            List<ArrayHelper> nextHelpers = GetAdjacentTiles(helper, end);

            // Sort by F-value
            nextHelpers.Sort((helper1, helper2) => helper1.F.CompareTo(helper2.F));
            foreach (var nextHelper in nextHelpers)
            {
                if (nextHelper.DeltaX == end.DeltaX && nextHelper.DeltaY == end.DeltaY)
                {
                    return true;
                }
                else
                {
                    if (Search(nextHelper, end))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public List<ArrayHelper> FindPath(ArrayHelper startTile, ArrayHelper endTile)
        {
            startTile.State = ArrayHelper.HelperState.OPEN;

            List<ArrayHelper> path = new List<ArrayHelper>();
            bool success = Search(startTile, endTile);
            if (success)
            {
                ArrayHelper helper = endTile;
                while (helper.ParentArrayHelper != null)
                {
                    path.Add(helper);
                    helper = helper.ParentArrayHelper;
                }
                path.Reverse();
            }
            return path;
        }
    }
}