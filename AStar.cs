public class AStar : MonoBehaviour
{
    //-------------------------------------------------------------------------------------------------------------------
    //-+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- VARIABLES -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+-
    //-------------------------------------------------------------------------------------------------------------------
    //===================================================================================================================
    [Header("NODES")]
    [SerializeField] private AStarNode[,] _allNodes;
    [SerializeField] private HashSet<AStarNode> _OPEN_nodes;
    [SerializeField] private HashSet<AStarNode> _CLOSED_nodes;
    [SerializeField] private AStarNode _startNode;
    [SerializeField] private AStarNode _targetNode;
    [SerializeField] private AStarNode _currentNode;
    [SerializeField] private List<AStarNode> _neighborNodes;
    //===================================================================================================================
    [Header("NODE GRID")]
    [SerializeField] private int _nodeGridHeight;
    [SerializeField] private int _nodeGridWidth;
    [SerializeField] private Vector3 _gridOriginPosition;
    //===================================================================================================================
    [Header("ASTAR ALGORITHM MODS")]
    [SerializeField] private int _costOfDiagonal; // = 14
    [SerializeField] private int _costOfHorizontal; // = 10
    [SerializeField] private int _weightOfGCost; // = 1
    [SerializeField] private int _weightOfHCost; // = 1
    //===================================================================================================================
    [Header("ASTAR DEBUG PREFERENCES")]
    [SerializeField] private bool _debuggingActive;
    [SerializeField] private bool _runAStarInstantly;
    [SerializeField] private int _maxIterationsAllowed;
    [SerializeField] private float _eachNeighborCheckDelay;
    [SerializeField] private float _eachCheckDelay;
    [SerializeField] private float _debugLineDuration;
    [SerializeField] private int _currentIterationCount;
    //===================================================================================================================
    [Header("REFERENCES")]
    [SerializeField] private Tilemap _floor;
    //===================================================================================================================

    //-------------------------------------------------------------------------------------------------------------------
    //-+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- BASE FUNCTIONS -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+-
    //-------------------------------------------------------------------------------------------------------------------
    private void OnEnable()
    {
        EventsManager.instance.gameEvents.OnSetupNewAStarGrid += SetupAStar;
    }
    private void OnDisable()
    {
        EventsManager.instance.gameEvents.OnSetupNewAStarGrid -= SetupAStar;
    }

    //-------------------------------------------------------------------------------------------------------------------
    //-+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- CLASS FUNCTIONS -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+- -+-
    //-------------------------------------------------------------------------------------------------------------------
    //===================================================================================================================
    // SETUP
    //===================================================================================================================
    public void SetupAStar(int height, int width, Vector3 origin, Tilemap floor)
    {
        _floor = floor;
        _nodeGridHeight = height;
        _nodeGridWidth = width;
        _gridOriginPosition = origin;
        _allNodes = new AStarNode[_nodeGridWidth, _nodeGridHeight];
        _OPEN_nodes = new HashSet<AStarNode>();
        _CLOSED_nodes = new HashSet<AStarNode>();
        _neighborNodes = new List<AStarNode>();
        for (int x = 0; x < _nodeGridWidth; x++)
        {
            for (int y = 0; y < _nodeGridHeight; y++)
            {
                _allNodes[x, y] = new AStarNode(x, y, AStarState.UNTRAVERSABLE);
                MakeTraversableIfTileNotNull(_allNodes[x, y], _floor);

                if (_debuggingActive)
                {
                    _allNodes[x, y].f_cost_text = Utility.CreateWorldText((_allNodes[x, y].x + _allNodes[x, y].y).ToString(), null, _allNodes[x, y].F_cost().ToString(), GetWorldPos(x, y) + new Vector3(1, 1) * .5f, 250, Color.white, TextAnchor.MiddleCenter, TextAlignment.Center, 0, .015f);
                    _allNodes[x, y].g_cost_text = Utility.CreateWorldText((_allNodes[x, y].x + _allNodes[x, y].y).ToString(), null, _allNodes[x, y].g_cost.ToString(), GetWorldPos(x, y) + new Vector3(.3f, .7f), 200, Color.white, TextAnchor.LowerRight, TextAlignment.Center, 0, 0.01f);
                    _allNodes[x, y].h_cost_text = Utility.CreateWorldText((_allNodes[x, y].x + _allNodes[x, y].y).ToString(), null, _allNodes[x, y].h_cost.ToString(), GetWorldPos(x, y) + new Vector3(1, 1) * .7f, 200, Color.white, TextAnchor.LowerLeft, TextAlignment.Center, 0, 0.01f);
                    Debug.DrawLine(GetWorldPos(x, y), GetWorldPos(x, y + 1), Color.white, _debugLineDuration);
                    Debug.DrawLine(GetWorldPos(x, y), GetWorldPos(x + 1, y), Color.white, _debugLineDuration);
                }
            }
        }
        if (_debuggingActive)
        {
            Debug.DrawLine(GetWorldPos(0, _nodeGridHeight), GetWorldPos(_nodeGridWidth, _nodeGridHeight), Color.white, _debugLineDuration);
            Debug.DrawLine(GetWorldPos(_nodeGridWidth, 0), GetWorldPos(_nodeGridWidth, _nodeGridHeight), Color.white, _debugLineDuration);
        }
    }

    //===================================================================================================================
    // RUN ASTAR
    //===================================================================================================================
    public List<Vector3> TryRunAStar(AStarNode startNode, AStarNode targetNode)
    {
        List<Vector3> ret = new List<Vector3>();
        ResetAStar();
        if (_runAStarInstantly) ret = RunAStar(startNode, targetNode);
        // else StartCoroutine(RunAStarCoroutine(startNode, targetNode, out ret));
        return ret;
    }
    private List<Vector3> RunAStar(AStarNode startNode, AStarNode targetNode)
    {
        // GUARD
        if (startNode == null || targetNode == null) { Debug.Log("StartNode or TargetNode is null"); return null; }
        // VARS
        _currentIterationCount = 0;
        _startNode = startNode;
        _targetNode = targetNode;
        // SETUP START NODE
        _startNode.g_cost = _weightOfGCost * GetNodeG_Cost(startNode, startNode);
        _startNode.h_cost = _weightOfHCost * GetNodeH_Cost(startNode, targetNode);
        if (_debuggingActive) UpdateNodeDebugText(_startNode);
        _OPEN_nodes.Add(_startNode);
        // RUN A* 
        while (true)
        {
            // GUARD
            _currentIterationCount++;
            if (_currentIterationCount > _maxIterationsAllowed)
            {
                Debug.Log("Exceeded maximum iterations");
                return null;
            }
            // GET NEW CURRENT NODE
            _currentNode = GetNodeWithLowestF_Cost(_OPEN_nodes);
            // UPDATE CURRENT NODE
            RemoveNodeFromOpenAddToClosed(_currentNode);
            // TRY RETURN
            if (_currentNode == _targetNode)
            {
                if (_debuggingActive) LightUpPath(GetQuickestPath(_currentNode));
                return GetQuickestPositionPath(_currentNode);
            }
            // GET NEIGHBORS
            _neighborNodes = GetNeighborsOfNode(_currentNode);
            if (_neighborNodes == null) return null;
            // PROCESS NEIGHBORS
            foreach (AStarNode neighbor in _neighborNodes)
            {
                // SKIP BAD NODES
                if (neighbor.aStarState == AStarState.CLOSED || neighbor.aStarState == AStarState.UNTRAVERSABLE) continue;
                // TRY UPDATE NEIGHBORS
                if (neighbor.aStarState != AStarState.OPEN || PathToNeighborIsShorter(_currentNode, neighbor, _targetNode))
                {
                    // UPDATE NODE COSTS
                    neighbor.g_cost = _weightOfGCost * (_currentNode.g_cost + GetNodeG_Cost(neighbor, _currentNode));
                    neighbor.h_cost = _weightOfHCost * GetNodeH_Cost(neighbor, _targetNode);
                    if (_debuggingActive) UpdateNodeDebugText(neighbor);
                    // SET NEW PARENT
                    neighbor.parent = _currentNode;
                    // NEIGHBOR TO OPEN IF NOT ALREADY
                    if (neighbor.aStarState != AStarState.OPEN)
                    {
                        neighbor.aStarState = AStarState.OPEN;
                        _OPEN_nodes.Add(neighbor);
                    }
                }
            }
        }
    }

    //===================================================================================================================
    // GET POSITIONS
    //===================================================================================================================
    private Vector3 GetWorldPos(int x, int y)
    {
        return new Vector3(x, y) + _gridOriginPosition;
    }
    private Vector3 GetWorldPos(AStarNode node)
    {
        return GetWorldPos(node.x, node.y);
    }
    private Vector3Int GetIntWorldPos(AStarNode node)
    {
        int x = Mathf.FloorToInt(node.x + _gridOriginPosition.x);
        int y = Mathf.FloorToInt(node.y + _gridOriginPosition.y);
        return new Vector3Int(x, y, 0); // Assuming z = 0 for 2D tilemaps
    }

    //===================================================================================================================
    // GET NODES
    //===================================================================================================================
    private AStarNode GetNodeWithLowestF_Cost(HashSet<AStarNode> nodeHashSet)
    {
        if (nodeHashSet == null || nodeHashSet.Count == 0)
        {
            Debug.Log($"nodeHashSet is null or empty in GetNodeWithLowestF_Cost. Length: {nodeHashSet.Count}");
            return null;
        }
        AStarNode lowestF_Cost = null;
        // FIND LOWEST F COST
        foreach (AStarNode node in nodeHashSet)
        {
            if (lowestF_Cost == null || node.F_cost() <= lowestF_Cost.F_cost()) lowestF_Cost = node;
        }
        return lowestF_Cost;
    }
    private List<AStarNode> GetNeighborsOfNode(AStarNode node)
    {
        if (node == null) { Debug.Log("Node was null"); return null; }
        // VARS
        List<AStarNode> neighbors = new List<AStarNode>();
        AStarNode nextToBeAdded;
        // ADD NEIGHBORS
        nextToBeAdded = GetGridNode(node.x, node.y + 1); // UP
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x + 1, node.y); // RIGHT
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x + 1, node.y + 1); // UP RIGHT
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x, node.y - 1); // DOWN
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x - 1, node.y); // LEFT
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x - 1, node.y - 1); // DOWN LEFT
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x + 1, node.y - 1); // DOWN RIGHT
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        nextToBeAdded = GetGridNode(node.x - 1, node.y + 1); // UP LEFT
        if (nextToBeAdded != null) neighbors.Add(nextToBeAdded);
        return neighbors;
    }
    public AStarNode GetGridNode(int x, int y)
    {
        if (x >= 0 && y >= 0 && x < _nodeGridWidth && y < _nodeGridHeight)
        {
            return _allNodes[x, y];
        }
        return null;
    }
    public AStarNode GetGridNode(Vector3 worldPos)
    {
        int x, y;
        GetXY(worldPos, out x, out y);
        return GetGridNode(x, y);
    }

    //===================================================================================================================
    // MISC
    //===================================================================================================================
    private void GetXY(Vector3 worldPos, out int x, out int y)
    {
        x = Mathf.FloorToInt((worldPos - _gridOriginPosition).x);
        y = Mathf.FloorToInt((worldPos - _gridOriginPosition).y);
    }
    private void RemoveNodeFromOpenAddToClosed(AStarNode node)
    {
        if (node == null) return;
        _OPEN_nodes.Remove(node);
        _CLOSED_nodes.Add(node);
        node.aStarState = AStarState.CLOSED;
        if (_debuggingActive) SetNodeDebugTextColor(node, Color.gray);
    }
    private bool PathToNeighborIsShorter(AStarNode current, AStarNode neighbor, AStarNode targetNode)
    {
        if (neighbor.F_cost() > current.g_cost + GetNodeG_Cost(neighbor, current) + GetNodeH_Cost(neighbor, targetNode)) return true;
        return false;
    }
    private int GetNodeG_Cost(AStarNode node, AStarNode startNode)
    {
        if (node == null || startNode == null) return -1;

        int distanceX = Math.Abs(node.x - startNode.x);
        int distanceY = Math.Abs(node.y - startNode.y);

        int diagonalMoves = Math.Min(distanceX, distanceY); // MINIMUM NUMBER = MAX DIAG MOVES
        int horizontalMoves = Math.Abs(distanceX - distanceY); // LEFTOVER MOVES 

        return diagonalMoves * _costOfDiagonal + horizontalMoves * _costOfHorizontal;
    }
    private int GetNodeH_Cost(AStarNode node, AStarNode endNode)
    {
        if (node == null || endNode == null) return -1;

        int distanceX = Math.Abs(node.x - endNode.x);
        int distanceY = Math.Abs(node.y - endNode.y);

        int diagonalMoves = Math.Min(distanceX, distanceY); // MINIMUM NUMBER = MAX DIAG MOVES
        int horizontalMoves = Math.Abs(distanceX - distanceY); // LEFTOVER MOVES 

        return diagonalMoves * _costOfDiagonal + horizontalMoves * _costOfHorizontal;
    }

    //===================================================================================================================
    // RESET
    //===================================================================================================================
    private void ResetAStar()
    {
        _currentIterationCount = 0;
        for (int x = 0; x < _nodeGridWidth; x++)
        {
            for (int y = 0; y < _nodeGridHeight; y++)
            {
                if (_allNodes[x, y] != null)
                {
                    if (_allNodes[x, y].aStarState != AStarState.UNTRAVERSABLE)
                    {
                        _allNodes[x, y].aStarState = AStarState.UNTESTED;
                        _allNodes[x, y].g_cost = 0;
                        _allNodes[x, y].h_cost = 0;
                        _allNodes[x, y].parent = null;
                        if (_debuggingActive)
                        {
                            _allNodes[x, y].f_cost_text.text = "0";
                            _allNodes[x, y].g_cost_text.text = "0";
                            _allNodes[x, y].h_cost_text.text = "0";
                            SetNodeDebugTextColor(_allNodes[x, y], Color.white);
                        }
                    }
                }
            }
        }
        _OPEN_nodes?.Clear();
        _CLOSED_nodes?.Clear();
        _currentNode = null;
        _neighborNodes?.Clear();
    }

    //===================================================================================================================
    // SET NODES
    //===================================================================================================================
    private void SetStartingNode(AStarNode node)
    {
        // GUARD
        if (node == null || node.aStarState == AStarState.UNTRAVERSABLE) return;
        // RESET PREVIOUS STARTNODE COLOR
        if (_startNode != null) if (_debuggingActive) SetNodeDebugTextColor(_startNode, Color.white);
        // SET NEW STARTNODE COLOR
        if (_debuggingActive) SetNodeDebugTextColor(node, Color.blue);
        _startNode = node;
    }
    public void SetStartingNode(Vector3 pos)
    {
        SetStartingNode(GetGridNode(pos));
    }
    private void SetTargetNode(AStarNode node)
    {
        // GUARD
        if (node == null || node.aStarState == AStarState.UNTRAVERSABLE) return;
        // RESET PREVIOUS TARGETNODE COLOR
        if (_targetNode != null) if (_debuggingActive) SetNodeDebugTextColor(_targetNode, Color.white);
        // SET NEW TARGETNODE COLOR
        if (_debuggingActive) SetNodeDebugTextColor(node, Color.red);
        _targetNode = node;
    }
    public void SetTargetNode(Vector3 pos)
    {
        SetTargetNode(GetGridNode(pos));
    }

    //===================================================================================================================
    // PATH
    //===================================================================================================================
    private List<AStarNode> GetQuickestPath(AStarNode finalNode)
    {
        if (finalNode == null) return null;
        List<AStarNode> path = new();
        AStarNode current = finalNode;
        do
        {
            path.Add(current);
            current = current.parent;
        }
        while (current.parent != null);
        return path;
    }
    private List<Vector3> GetQuickestPositionPath(AStarNode finalNode)
    {
        if (finalNode == null) return null;
        List<Vector3> path = new();
        AStarNode current = finalNode;
        do
        {
            path.Add(new Vector3(current.x + .5f, current.y + .5f) + _gridOriginPosition);
            current = current.parent;
        }
        while (current.parent != null);
        return path;
    }
    private void LightUpPath(List<AStarNode> path)
    {
        if (path == null) return;
        foreach (AStarNode node in path)
        {
            SetNodeDebugTextColor(node, Color.yellow);
        }
        // SET ENDPOINT NODE COLOR
        SetNodeDebugTextColor(_startNode, Color.blue);
        SetNodeDebugTextColor(_targetNode, Color.red);
    }

    //===================================================================================================================
    // UPDATE NODES
    //===================================================================================================================
    private void SetNodeDebugTextColor(AStarNode node, Color color)
    {
        node.f_cost_text.color = color;
        node.g_cost_text.color = color;
        node.h_cost_text.color = color;
    }
    private void UpdateNodeDebugText(AStarNode node)
    {
        if (node == null) return;
        node.g_cost_text.text = node.g_cost.ToString();
        node.h_cost_text.text = node.h_cost.ToString();
        node.f_cost_text.text = node.F_cost().ToString();
    }
    private void MakeTraversableIfTileNotNull(AStarNode node, Tilemap tilemap)
    {
        Vector3Int tilePosition = GetIntWorldPos(node);
        if (tilemap.HasTile(tilePosition))
        {
            node.aStarState = AStarState.UNTESTED;
            // Debug.Log("Tile exists at position: " + tilePosition);
            // Instantiate(test, tilePosition, Quaternion.identity);
        }
        else if (!tilemap.HasTile(tilePosition))
        {
            Destroy(node.f_cost_text);
            Destroy(node.g_cost_text);
            Destroy(node.h_cost_text);
        }
    }

    //===================================================================================================================
    // ALGO MODS
    //===================================================================================================================
    public int CostOfDiagonal
    {
        set { _costOfDiagonal = value; }
    }
    public int CostOfHorizontal
    {
        set { _costOfHorizontal = value; }
    }
    public int WeightOfGCost
    {
        set { _weightOfGCost = value; }
    }
    public int WeightOfHCost
    {
        set { _weightOfHCost = value; }
    }

    //===================================================================================================================
    // ASTAR DEBUG PREFERENCES
    //===================================================================================================================
    public bool RunAStarInstantly
    {
        set { _runAStarInstantly = value; }
    }
    public float EachNeighborCheckDelay
    {
        set { _eachNeighborCheckDelay = value; }
    }
    public float EachCheckDelay
    {
        set { _eachCheckDelay = value; }
    }
    public bool DebuggingActive
    {
        set { _debuggingActive = value; }
    }
}