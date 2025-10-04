using Godot;
using System;
using System.Collections.Generic;

public sealed partial class ArmyRange2D : CharacterBody2D
{
    // === Scene refs ===
    private NavigationAgent2D _agent;

    // === Sampling / overlay ===
    [Export] public float Cell { get; private set; } = 32f;      // шаг семплирования (px)
    [Export] public Rect2 WorldBounds { get; private set; } = new Rect2(-2000, -2000, 4000, 4000);

    // Заливка и сглаживание
    [Export] public int GrowMask { get; private set; } = 1;      // дилатация (px)
    [Export] public int ShrinkMask { get; private set; } = 0;    // эрозия (px) -> вызывается как GrowMask(-n)
    [Export] public float OpaqueEpsilon { get; private set; } = 2f; // точность в OpaqueToPolygons
    [Export] public int ChaikinIterations { get; private set; } = 2;
    [Export] public float ChaikinTension { get; private set; } = 0.25f;

    // Цвета
    [Export] public Color FillColor { get; private set; } = new Color(0f, 0.6f, 1f, 0.18f);
    [Export] public Color OutlineColor { get; private set; } = new Color(0f, 0.6f, 1f, 0.65f);
    [Export] public float OutlineWidth { get; private set; } = 2f;

    // === Movement budget ===
    [Export] public float MaxBudget { get; private set; } = 400f;     // очки на ход
    [Export] public float BudgetSlackAbs { get; private set; } = 32f;  // фикс. допуск (px-стоимость)
    [Export] public float BudgetSlackPct { get; private set; } = 0.10f;// доля допуска

    // Интеграция стоимости
    [Export] public float PathCostSampleStep { get; private set; } = 0f; // 0 => авто: Cell * 0.5f
    [Export] public float CostProjectEps { get; private set; } = 1.0f;   // сдвиг семпла внутрь сегмента
    [Export] public int OwnerDebounce { get; private set; } = 1;         // подтверждение смены региона

    // === Movement params ===
    [Export] public float MoveSpeed { get; private set; } = 220f;
    [Export] public float StopDistance { get; private set; } = 4f;

    // Debug/UX
    [Export] public bool DrawPathPreview { get; private set; } = true;

    // === State ===
    private float _budgetLeft;
    private bool _isMoving;
    private float _plannedCost;
    private Vector2 _lastSafeVelocity = Vector2.Zero;

    private HashSet<Vector2> _reachableWorld = new();
    private readonly List<Vector2[]> _polygonsLocal = new();
    private Vector2[] _currentPath = Array.Empty<Vector2>();

    // ======================================================================

    public override void _Ready()
    {
        _agent = GetNode<NavigationAgent2D>("NavigationAgent2D");
        _agent.AvoidanceEnabled = true;
        _agent.VelocityComputed += OnVelocityComputed;

        _budgetLeft = MaxBudget;

        NavigationServer2D.MapChanged += OnNavMapChanged;
        CallDeferred(nameof(RecomputeAndRedraw));
    }

    public override void _ExitTree()
    {
        NavigationServer2D.MapChanged -= OnNavMapChanged;
        if (_agent != null) _agent.VelocityComputed -= OnVelocityComputed;
    }

    private void OnNavMapChanged(Rid map)
    {
        if (map == _agent.GetNavigationMap())
            RecomputeAndRedraw();
    }

    // ======================================================================
    // TURN CONTROL

    public void BeginNewTurn()
    {
        _budgetLeft = MaxBudget;
        RecomputeAndRedraw();
    }

    // ======================================================================
    // RANGE + POLYGON

    public void RecomputeAndRedraw()
    {
        var start = GlobalPosition;
        _reachableWorld = ComputeReachable(WorldBounds, start, _budgetLeft);
        BuildFillPolygonsFromReachable();
        QueueRedraw();
    }

    private HashSet<Vector2> ComputeReachable(Rect2 worldBounds, Vector2 start, float budget)
    {
        var reached = new HashSet<Vector2>();
        var best = new Dictionary<Vector2, float>();
        var open = new PriorityQueue<Vector2, float>();

        var mapRid = _agent.GetNavigationMap();
        if (!mapRid.IsValid) return reached;

        var startSnapped = Snap(start);
        best[startSnapped] = 0f;
        open.Enqueue(startSnapped, 0f);

        while (open.Count > 0)
        {
            open.TryDequeue(out var p, out var costSoFar);
            if (costSoFar > budget) continue;
            reached.Add(p);

            foreach (var n in Neighbors8(p))
            {
                if (!worldBounds.HasPoint(n)) continue;
                if (!IsNavigable(mapRid, n)) continue;

                var stepCost = ComputeSegmentCost(mapRid, p, n);
                if (float.IsInfinity(stepCost) || stepCost <= 0f) continue;

                var newCost = costSoFar + stepCost;
                if (newCost > budget) continue;

                if (!best.TryGetValue(n, out var old) || newCost < old)
                {
                    best[n] = newCost;
                    open.Enqueue(n, newCost);
                }
            }
        }

        return reached;
    }

    private void BuildFillPolygonsFromReachable()
    {
        _polygonsLocal.Clear();

        var originX = Mathf.Floor(WorldBounds.Position.X / Cell) * Cell;
        var originY = Mathf.Floor(WorldBounds.Position.Y / Cell) * Cell;
        var cols = Mathf.CeilToInt(WorldBounds.Size.X / Cell) + 1;
        var rows = Mathf.CeilToInt(WorldBounds.Size.Y / Cell) + 1;

        var bm = new Bitmap();
        bm.Create(new Vector2I(cols, rows));

        for (var j = 0; j < rows; j++)
        {
            var y = originY + j * Cell;
            for (var i = 0; i < cols; i++)
            {
                var x = originX + i * Cell;
                var snapped = Snap(new Vector2(x, y));

                var on = _reachableWorld.Contains(snapped);
                if (!on)
                {
                    foreach (var rp in _reachableWorld)
                    {
                        if (rp.DistanceTo(snapped) <= Cell * 0.5f) { on = true; break; }
                    }
                }
                bm.SetBitv(new Vector2I(i, j), on);
            }
        }

        var rect = new Rect2I(0, 0, cols, rows);
        if (GrowMask > 0) bm.GrowMask(GrowMask, rect);
        if (ShrinkMask > 0) bm.GrowMask(-ShrinkMask, rect); // эрозия

        var polys = bm.OpaqueToPolygons(rect, OpaqueEpsilon);

        foreach (var poly in polys)
        {
            var vertsLocal = new Vector2[poly.Length];
            for (var k = 0; k < poly.Length; k++)
            {
                var world = new Vector2(originX, originY) + poly[k] * Cell;
                vertsLocal[k] = ToLocal(world);
            }

            vertsLocal = RdpSimplify(vertsLocal, Cell * 0.5f);
            if (ChaikinIterations > 0)
                vertsLocal = Chaikin(vertsLocal, ChaikinIterations, ChaikinTension);

            if (vertsLocal.Length >= 3)
                _polygonsLocal.Add(vertsLocal);
        }
    }

    // ======================================================================
    // INPUT / MOVE

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is InputEventMouseButton mb && mb.ButtonIndex == MouseButton.Left && mb.Pressed)
            TryMoveTo(GetGlobalMousePosition());
    }

    public void TryMoveTo(Vector2 target)
    {
        if (!IsInReachableArea(target)) return;

        var map = _agent.GetNavigationMap();
        if (!map.IsValid) return;

        // прижимаем цель к навмешу, чтобы не целиться вне карты
        target = NavigationServer2D.MapGetClosestPoint(map, target);

        _plannedCost = 0f;
        _isMoving = false;
        _lastSafeVelocity = Vector2.Zero;
        _currentPath = Array.Empty<Vector2>();

        _agent.TargetPosition = target; // агент посчитает путь и дернёт VelocityComputed
    }

    private bool IsInReachableArea(Vector2 worldPoint)
    {
        var local = ToLocal(worldPoint);
        foreach (var poly in _polygonsLocal)
            if (Geometry2D.IsPointInPolygon(local, poly)) return true;
        return false;
    }

    private void OnVelocityComputed(Vector2 safeVelocity)
    {
        if (!_agent.IsNavigationFinished())
        {
            var path = _agent.GetCurrentNavigationPath();
            if (path != null && path.Length > 1)
            {
                if (!_isMoving)
                {
                    var cost = ComputePathCost(_agent.GetNavigationMap(), path);
                    var slack = BudgetSlackAbs + _budgetLeft * BudgetSlackPct;

                    if (float.IsInfinity(cost) || cost > _budgetLeft + slack)
                    {
                        _agent.TargetPosition = GlobalPosition;
                        _lastSafeVelocity = Vector2.Zero;
                        _currentPath = Array.Empty<Vector2>();
                        return;
                    }

                    _plannedCost = cost;
                    _isMoving = true;
                }

                _lastSafeVelocity = safeVelocity;
                _currentPath = path;
            }
        }
        else
        {
            _lastSafeVelocity = Vector2.Zero;
            _currentPath = Array.Empty<Vector2>();
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_isMoving)
        {
            var next = _agent.GetNextPathPosition();
            var toNext = next - GlobalPosition;

            if (toNext.Length() > StopDistance)
            {
                var desired = toNext.Normalized() * MoveSpeed;
                _agent.Velocity = desired; // вызовет VelocityComputed в этот кадр
            }
            else
            {
                _agent.Velocity = Vector2.Zero;
            }

            Velocity = _lastSafeVelocity;
            MoveAndSlide();

            if (_agent.IsNavigationFinished())
            {
                _isMoving = false;
                Velocity = Vector2.Zero;
                _lastSafeVelocity = Vector2.Zero;

                _budgetLeft -= _plannedCost;
                if (_budgetLeft < 0f) _budgetLeft = 0f;

                RecomputeAndRedraw();
            }
        }
        else
        {
            _agent.Velocity = Vector2.Zero;
            Velocity = Vector2.Zero;
            MoveAndSlide();
        }
    }

    // ======================================================================
    // COSTS (с проекцией на навмеш и «дебаунсом» смены региона)

    private float ComputeSegmentCost(Rid map, Vector2 from, Vector2 to)
    {
        var segLen = from.DistanceTo(to);
        if (segLen <= 0.0001f) return 0f;

        var step = PathCostSampleStep > 0f ? PathCostSampleStep : Mathf.Max(4f, Cell * 0.5f);
        var samples = Mathf.Max(1, Mathf.CeilToInt(segLen / step));
        var inv = 1f / samples;
        var dir = (to - from).Normalized();

        var p0 = NavigationServer2D.MapGetClosestPoint(map, from + dir * CostProjectEps);
        var ownerCur = NavigationServer2D.MapGetClosestPointOwner(map, p0);
        if (!ownerCur.IsValid) return float.PositiveInfinity;

        var total = 0f;
        var subLen = segLen * inv;

        var pendingOwner = ownerCur;
        var sameCount = 0;

        for (var s = 1; s <= samples; s++)
        {
            var t = s * inv;
            var p = from.Lerp(to, t);

            var cp = NavigationServer2D.MapGetClosestPoint(map, p + dir * CostProjectEps);
            var owner = NavigationServer2D.MapGetClosestPointOwner(map, cp);
            if (!owner.IsValid) owner = ownerCur;

            if (owner != ownerCur)
            {
                if (owner == pendingOwner)
                {
                    sameCount++;
                    if (sameCount >= Math.Max(1, OwnerDebounce))
                    {
                        total += NavigationServer2D.RegionGetEnterCost(owner);
                        ownerCur = owner;
                        sameCount = 0;
                    }
                }
                else
                {
                    pendingOwner = owner;
                    sameCount = 1;
                }
            }
            else
            {
                pendingOwner = ownerCur;
                sameCount = 0;
            }

            var travel = NavigationServer2D.RegionGetTravelCost(ownerCur);
            total += subLen * travel;
        }

        return total;
    }

    private float ComputePathCost(Rid map, Vector2[] pts)
    {
        if (pts == null || pts.Length < 2) return float.PositiveInfinity;

        var total = 0f;
        for (var i = 1; i < pts.Length; i++)
        {
            var c = ComputeSegmentCost(map, pts[i - 1], pts[i]);
            if (float.IsInfinity(c)) return float.PositiveInfinity;
            total += c;
        }
        return total;
    }

    // ======================================================================
    // DRAW

    public override void _Draw()
    {
        foreach (var poly in _polygonsLocal)
        {
            if (poly.Length < 3) continue;
            DrawPolygon(poly, new Color[] { FillColor });
            for (var i = 0; i < poly.Length; i++)
            {
                var a = poly[i];
                var b = poly[(i + 1) % poly.Length];
                DrawLine(a, b, OutlineColor, OutlineWidth);
            }
        }

        if (DrawPathPreview && _currentPath != null && _currentPath.Length > 1)
        {
            var local = new Vector2[_currentPath.Length];
            for (var i = 0; i < _currentPath.Length; i++)
                local[i] = ToLocal(_currentPath[i]);

            var negateOutline = OutlineColor;
            negateOutline.H = (negateOutline.H + 0.5f) % 1;

            DrawPolyline(local, negateOutline, OutlineWidth);
        }
    }

    // ======================================================================
    // Helpers

    private Vector2 Snap(Vector2 v)
        => new(Mathf.Round(v.X / Cell) * Cell, Mathf.Round(v.Y / Cell) * Cell);

    private IEnumerable<Vector2> Neighbors8(Vector2 p)
    {
        var d = Cell;
        yield return p + new Vector2(d, 0);
        yield return p + new Vector2(-d, 0);
        yield return p + new Vector2(0, d);
        yield return p + new Vector2(0, -d);
        yield return p + new Vector2(d, d);
        yield return p + new Vector2(-d, d);
        yield return p + new Vector2(d, -d);
        yield return p + new Vector2(-d, -d);
    }

    private bool IsNavigable(Rid mapRid, Vector2 world)
    {
        var closest = NavigationServer2D.MapGetClosestPoint(mapRid, world);
        var dist = world.DistanceTo(closest);
        return dist <= Cell * 0.75f;
    }

    // --- contour utils ---

    private static Vector2[] RdpSimplify(Vector2[] pts, float epsilon)
    {
        if (pts.Length < 3) return pts;

        var stack = new Stack<(int a, int b)>();
        var keep = new bool[pts.Length];
        keep[0] = keep[pts.Length - 1] = true;
        stack.Push((0, pts.Length - 1));

        while (stack.Count > 0)
        {
            var (a, b) = stack.Pop();
            var maxDist = 0f;
            var idx = -1;

            var ab = pts[b] - pts[a];
            var abLen2 = ab.LengthSquared();

            for (var i = a + 1; i < b; i++)
            {
                var ap = pts[i] - pts[a];
                var t = abLen2 > 0f ? Mathf.Clamp(ap.Dot(ab) / abLen2, 0f, 1f) : 0f;
                var proj = pts[a] + ab * t;
                var d = pts[i].DistanceTo(proj);
                if (d > maxDist) { maxDist = d; idx = i; }
            }

            if (maxDist > epsilon && idx >= 0)
            {
                keep[idx] = true;
                stack.Push((a, idx));
                stack.Push((idx, b));
            }
        }

        var list = new List<Vector2>(pts.Length);
        for (var i = 0; i < pts.Length; i++)
            if (keep[i]) list.Add(pts[i]);
        return list.ToArray();
    }

    private static Vector2[] Chaikin(Vector2[] poly, int iterations, float tension = 0.25f)
    {
        var pts = poly;
        for (var it = 0; it < iterations; it++)
        {
            var res = new List<Vector2>(pts.Length * 2);
            for (var i = 0; i < pts.Length; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % pts.Length];
                var q = a * (1f - tension) + b * tension;
                var r = a * tension + b * (1f - tension);
                res.Add(q);
                res.Add(r);
            }
            pts = res.ToArray();
        }
        return pts;
    }
}