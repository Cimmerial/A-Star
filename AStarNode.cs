using UnityEngine;

public class AStarNode
{
    public int x;
    public int y;
    public AStarState aStarState;
    public AStarNode parent;
    public int F_cost() => g_cost + h_cost;
    public int g_cost;
    public int h_cost;

    public TextMesh f_cost_text;
    public TextMesh g_cost_text;
    public TextMesh h_cost_text;

    public AStarNode(int x, int y, AStarState aStarState)
    {
        this.x = x;
        this.y = y;
        this.aStarState = aStarState;
        parent = null;
        g_cost = 0;
        h_cost = 0;
    }
}