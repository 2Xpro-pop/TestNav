using Godot;
using System;
using System.Collections.Generic;
using TestNav;

public sealed partial class ExplorationHero : CharacterBody2D
{
    // --- Сетка и область построения графа ---
    [Export] public float CellSize { get; set; } = 16f;
    [Export] public Rect2 WorldBounds { get; set; } = new Rect2(-1024, -1024, 2048, 2048);

    // --- Бюджет и стоимость шага ---
    [Export] public float Budget { get; set; } = 100f;
    [Export] public bool WeightByDistance { get; set; } = true;

    // --- Движение ---
    [ExportCategory("Movement")]
    [Export] public float MoveSpeed { get; set; } = 220f;
    [Export] public float StopDistance { get; set; } = 4f;

    // --- Проверка коллизий ---
    [ExportCategory("Collision Query")]
    [Export] public uint ObstacleMask { get; set; } = 1;
    [Export] public float Clearance { get; set; } = 2f;
    [Export] public int SegmentChecks { get; set; } = 2;

    // --- Отрисовка зоны и дерева путей ---
    [ExportCategory("Debug Draw")]
    [Export] public float DotRadius { get; set; } = 3f;
    [Export] public Color DotFill { get; set; } = new Color(0.1f, 0.8f, 1f, 0.85f);
    [Export] public bool DrawTreeEdges { get; set; } = false;
    [Export] public Color EdgeColor { get; set; } = new Color(0.25f, 0.9f, 0.9f, 0.6f);
    [Export] public float EdgeWidth { get; set; } = 1.5f;

    // --- Границы/заливка области достижимости ---
    [Export] public bool DrawPolygonBounds { get; set; } = false;
    [Export] public Color BoundsFill { get; set; } = new Color(0.0f, 0.8f, 1f, 0.14f);
    [Export] public Color BoundsOutline { get; set; } = new Color(0.0f, 0.8f, 1f, 0.8f);
    [Export] public float BoundsOutlineWidth { get; set; } = 2f;

    // Как собирать маску для контуров
    [ExportCategory("Bounds Build")]
    [Export] public int BoundsHalo { get; set; } = 1;      // «ореол» вокруг каждой достижимой клетки (px в bitmap'е)
    [Export] public int BoundsGrow { get; set; } = 1;      // доп. расширение (дилатация)
    [Export] public int BoundsShrink { get; set; } = 0;    // эрозия после расширения
    [Export] public float BoundsEpsilon { get; set; } = 1.5f; // точность аппроксимации в OpaqueToPolygons

    // --- Отрисовка стрелки маршрута ---
    [ExportCategory("Path Preview")]
    [Export] public Color PathColor { get; set; } = new Color(0.2f, 0.9f, 1f, 0.9f);
    [Export] public float PathWidth { get; set; } = 2.5f;
    [Export] public float ArrowHeadLength { get; set; } = 10f;
    [Export] public float ArrowHeadHalfWidth { get; set; } = 5f;

    // --- Граф и результаты ---
    private AStar2D _astar = default!;
    private readonly Dictionary<Vector2I, long> _gridToId = new();
    private readonly Dictionary<long, Vector2> _idToWorld = new();

    private readonly HashSet<long> _reachableIds = new();
    private readonly Dictionary<long, float> _bestCost = new();
    private readonly Dictionary<long, long> _parent = new();

    // Кеш сетки
    private Vector2 _gridOrigin;
    private int _cols, _rows;

    // --- Runtime: бюджет и движение ---
    private float _budgetLeft;
    private bool _isMoving;
    private Vector2[] _movePath = Array.Empty<Vector2>();
    private int _moveIndex;
    private int _stopAtIndex = int.MaxValue;

    // --- Preview при зажатой ПКМ ---
    private bool _isPreviewing;
    private bool _previewCancel;
    private Vector2[] _previewPath = Array.Empty<Vector2>();
    private Vector2 _previewTarget;

    // --- Полигональные контуры области достижимости (локальные координаты) ---
    private readonly List<Vector2[]> _boundsPolysLocal = new();

    // =====================================================================
    public override void _Ready()
    {
        _budgetLeft = Budget;
        RebuildAll();
    }

    public float GetBudgetForPoint(Vector2 point)
    {
        var snapped = Snap(point);
        var id = WorldToNodeId(snapped);
        if (id == -1) return float.PositiveInfinity;
        return _bestCost.TryGetValue(id, out var c) ? c : float.PositiveInfinity;
    }

    // =====================================================================
    // ГРАФ + ЗОНА

    public void RebuildAll()
    {
        BuildGridCache();
        BuildGraph();
        ComputeReachable();     // внутри пересоберём и контуры
        QueueRedraw();
    }

    private void BuildGridCache()
    {
        var ox = Mathf.Floor(WorldBounds.Position.X / CellSize) * CellSize;
        var oy = Mathf.Floor(WorldBounds.Position.Y / CellSize) * CellSize;
        _gridOrigin = new Vector2(ox, oy);

        _cols = Mathf.CeilToInt(WorldBounds.Size.X / CellSize) + 1;
        _rows = Mathf.CeilToInt(WorldBounds.Size.Y / CellSize) + 1;
    }

    private void BuildGraph()
    {
        _astar = new AStar2D();
        _gridToId.Clear();
        _idToWorld.Clear();

        var nextId = 1L;

        for (var j = 0; j < _rows; j++)
        {
            var y = _gridOrigin.Y + j * CellSize;
            for (var i = 0; i < _cols; i++)
            {
                var x = _gridOrigin.X + i * CellSize;
                var p = new Vector2(x, y);
                if (!IsPointFree(p)) continue;

                var id = nextId++;
                _gridToId[new Vector2I(i, j)] = id;
                _idToWorld[id] = p;
                _astar.AddPoint(id, p);
            }
        }

        var neigh = new Vector2I[]
        {
            new( 1, 0), new(-1, 0), new(0, 1), new(0,-1),
            new( 1, 1), new(-1, 1), new(1,-1), new(-1,-1)
        };

        foreach (var kv in _gridToId)
        {
            var ij = kv.Key;
            var id = kv.Value;

            foreach (var d in neigh)
            {
                var nIJ = ij + d;
                if (!_gridToId.TryGetValue(nIJ, out var nid)) continue;
                if (_astar.ArePointsConnected(id, nid)) continue;

                var a = _idToWorld[id];
                var b = _idToWorld[nid];
                if (!IsEdgeFree(a, b)) continue;

                _astar.ConnectPoints(id, nid, true);
            }
        }
    }

    private void ComputeReachable()
    {
        _reachableIds.Clear();
        _bestCost.Clear();
        _parent.Clear();

        var startId = WorldToNodeId(Snap(GlobalPosition));
        if (startId == -1) { _boundsPolysLocal.Clear(); return; }

        var open = new PriorityQueue<long, float>();
        _bestCost[startId] = 0f;
        _parent[startId] = startId;
        open.Enqueue(startId, 0f);

        while (open.Count > 0)
        {
            open.TryDequeue(out var id, out var costSoFar);
            if (costSoFar > _budgetLeft) continue;

            _reachableIds.Add(id);

            var conns = _astar.GetPointConnections(id);
            foreach (var nid in conns)
            {
                var a = _idToWorld[id];
                var b = _idToWorld[nid];

                var step = StepCost(a, b);
                if (float.IsInfinity(step) || step <= 0f) continue;

                var newCost = costSoFar + step;
                if (newCost > _budgetLeft) continue;

                if (!_bestCost.TryGetValue(nid, out var old) || newCost < old)
                {
                    _bestCost[nid] = newCost;
                    _parent[nid] = id;
                    open.Enqueue(nid, newCost);
                }
            }
        }

        // построим контуры для режима DrawPolygonBounds
        BuildBoundsPolygonsFromReachable();
    }

    // =====================================================================
    // ВВОД

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is InputEventMouseButton mb && mb.ButtonIndex == MouseButton.Right)
        {
            if (mb.Pressed)
            {
                _isPreviewing = true;
                _previewCancel = false;
                UpdatePreview(GetGlobalMousePosition());
            }
            else
            {
                if (_isPreviewing)
                {
                    if (!_previewCancel && _previewPath.Length > 1)
                        StartMoveFromPreview();
                    _isPreviewing = false;
                    _previewPath = Array.Empty<Vector2>();
                    QueueRedraw();
                }
            }
        }

        if (e is InputEventMouseMotion && _isPreviewing)
            UpdatePreview(GetGlobalMousePosition());

        if (e is InputEventKey ks && ks.Pressed && !ks.Echo && ks.Keycode == Key.S)
            if (_isPreviewing) _previewCancel = true;

        if (e is InputEventKey k && k.Pressed && !k.Echo && k.Keycode == Key.Space)
            if (_isMoving) RequestStopAtNearestNode();
    }

    // =====================================================================
    // ПРЕВЬЮ / ДВИЖЕНИЕ

    private void UpdatePreview(Vector2 targetWorld)
    {
        _previewTarget = targetWorld;

        var startId = _astar.GetClosestPoint(Snap(GlobalPosition));
        if (startId == -1) { _previewPath = Array.Empty<Vector2>(); QueueRedraw(); return; }

        var endId = _astar.GetClosestPoint(Snap(targetWorld));
        if (endId == -1) { _previewPath = Array.Empty<Vector2>(); QueueRedraw(); return; }

        var path = _astar.GetPointPath(startId, endId, true);
        _previewPath = ClampPathByBudget(path, _budgetLeft);
        QueueRedraw();
    }

    private void StartMoveFromPreview()
    {
        _movePath = _previewPath;
        _moveIndex = 1;
        _stopAtIndex = int.MaxValue;
        _isMoving = _movePath.Length > 1;
    }

    private Vector2[] ClampPathByBudget(Vector2[] worldPath, float budgetLeft)
    {
        if (worldPath == null || worldPath.Length < 2) return Array.Empty<Vector2>();

        var acc = 0f;
        var lastOk = 0;
        for (var i = 1; i < worldPath.Length; i++)
        {
            var c = StepCost(worldPath[i - 1], worldPath[i]);
            if (float.IsInfinity(c)) break;
            var next = acc + c;
            if (next > budgetLeft) break;
            acc = next;
            lastOk = i;
        }

        var span = lastOk + 1;
        var result = new Vector2[span];
        Array.Copy(worldPath, result, span);
        return result;
    }

    public override void _PhysicsProcess(double delta)
    {
        if (!_isMoving)
        {
            Velocity = Vector2.Zero;
            MoveAndSlide();
            return;
        }

        if (_moveIndex >= _movePath.Length)
        {
            FinishMovement();
            return;
        }

        var next = _movePath[_moveIndex];
        var toNext = next - GlobalPosition;

        if (toNext.Length() > StopDistance)
        {
            Velocity = toNext.Normalized() * MoveSpeed;
            MoveAndSlide();
        }
        else
        {
            var prev = _movePath[_moveIndex - 1];
            var step = StepCost(prev, next);
            _budgetLeft -= step;
            if (_budgetLeft < 0f) _budgetLeft = 0f;

            _moveIndex++;
            if (_moveIndex > _stopAtIndex || _moveIndex >= _movePath.Length)
                FinishMovement();

            ComputeReachable();
            QueueRedraw();
        }
    }

    private void RequestStopAtNearestNode()
    {
        if (!_isMoving || _movePath.Length < 2) return;

        var best = _moveIndex;
        var bestDist = float.MaxValue;

        for (var i = _moveIndex; i < _movePath.Length; i++)
        {
            var d = GlobalPosition.DistanceTo(_movePath[i]);
            if (d < bestDist) { bestDist = d; best = i; }
        }

        _stopAtIndex = Math.Max(best, _moveIndex);
    }

    private void FinishMovement()
    {
        _isMoving = false;
        Velocity = Vector2.Zero;
        MoveAndSlide();
        ComputeReachable();
        QueueRedraw();
    }

    // =====================================================================
    // СТОИМОСТЬ ШАГА

    private float StepCost(Vector2 from, Vector2 to)
    {
        var regFrom = GetRegionAtPoint(from);
        var regTo = GetRegionAtPoint(to);

        var edgeLen = WeightByDistance ? from.DistanceTo(to) : 1f;

        var travelFrom = regFrom?.TravelCost ?? 1f;
        var travelTo = regTo?.TravelCost ?? 1f;
        var travelCost = (travelFrom + travelTo) * 0.5f;

        var cost = travelCost * edgeLen;
        if (regTo != null && regTo != regFrom)
            cost += regTo.EnterCost;

        return MathF.Max(cost, 1e-4f);
    }

    // =====================================================================
    // КОЛЛИЗИИ

    private bool IsPointFree(Vector2 p)
    {
        var space = GetWorld2D().DirectSpaceState;

        var q = new PhysicsPointQueryParameters2D
        {
            CollideWithBodies = true,
            CollideWithAreas = true,
            CollisionMask = ObstacleMask,
            Exclude = [GetRid()],
            Position = p
        };

        var hits = space.IntersectPoint(q, 1);
        return hits.Count == 0;
    }

    private bool IsEdgeFree(Vector2 a, Vector2 b)
    {
        var space = GetWorld2D().DirectSpaceState;

        if (RayHitsObstacle(space, a, b)) return false;

        var dir = (b - a).Normalized();
        var perp = new Vector2(-dir.Y, dir.X);
        var off = perp * Clearance;

        if (Clearance > 0f)
        {
            if (RayHitsObstacle(space, a + off, b + off)) return false;
            if (RayHitsObstacle(space, a - off, b - off)) return false;
        }

        for (var i = 1; i <= SegmentChecks; i++)
        {
            var t = (float)i / (SegmentChecks + 1);
            var p = a.Lerp(b, t);
            if (!IsPointFree(p)) return false;
        }

        return true;
    }

    private bool RayHitsObstacle(PhysicsDirectSpaceState2D space, Vector2 from, Vector2 to)
    {
        var rq = PhysicsRayQueryParameters2D.Create(from, to);
        rq.CollisionMask = ObstacleMask;
        rq.CollideWithAreas = true;
        rq.Exclude = [GetRid()];

        var res = space.IntersectRay(rq);
        return res.Count > 0;
    }

    // =====================================================================
    // РЕГИОНЫ-ПОКРЫТИЯ

    private CustomNavigationRegion2D? GetRegionAtPoint(Vector2 p)
    {
        var space = GetWorld2D().DirectSpaceState;

        var q = new PhysicsPointQueryParameters2D
        {
            CollideWithBodies = false,
            CollideWithAreas = true,
            Position = p
        };

        var hits = space.IntersectPoint(q, 16);
        CustomNavigationRegion2D? best = null;

        foreach (var h in hits)
        {
            var node = h["collider"].Obj as Node;
            var reg = node as CustomNavigationRegion2D;
            if (reg == null) continue;
            if (best == null || reg.TravelCost < best.TravelCost)
                best = reg;
        }
        return best;
    }

    // =====================================================================
    // УТИЛИТЫ СЕТКИ

    public Vector2 Snap(Vector2 target)
        => new(Mathf.Round(target.X / CellSize) * CellSize, Mathf.Round(target.Y / CellSize) * CellSize);

    private long WorldToNodeId(Vector2 world)
    {
        var snapped = Snap(world);
        var i = Mathf.RoundToInt((snapped.X - _gridOrigin.X) / CellSize);
        var j = Mathf.RoundToInt((snapped.Y - _gridOrigin.Y) / CellSize);
        var ij = new Vector2I(i, j);
        return _gridToId.TryGetValue(ij, out var id) ? id : -1;
    }

    // =====================================================================
    // ПОСТРОЕНИЕ КОНТУРОВ

    private void BuildBoundsPolygonsFromReachable()
    {
        _boundsPolysLocal.Clear();
        if (!DrawPolygonBounds) return;

        var cols = _cols;
        var rows = _rows;

        var bm = new Bitmap();
        bm.Create(new Vector2I(cols, rows));

        // 1) Отмечаем клетки по достижимым узлам + "ореол" вокруг
        foreach (var id in _reachableIds)
        {
            var p = _idToWorld[id];
            var i = Mathf.RoundToInt((p.X - _gridOrigin.X) / CellSize);
            var j = Mathf.RoundToInt((p.Y - _gridOrigin.Y) / CellSize);

            for (int dy = -BoundsHalo; dy <= BoundsHalo; dy++)
                for (int dx = -BoundsHalo; dx <= BoundsHalo; dx++)
                {
                    // круглый ореол (можно заменить на квадрат — убрать проверку по радиусу)
                    if (dx * dx + dy * dy > BoundsHalo * BoundsHalo) continue;

                    var ii = i + dx;
                    var jj = j + dy;
                    if ((uint)ii < (uint)cols && (uint)jj < (uint)rows)
                        bm.SetBit(ii, jj, true);
                }
        }

        var fullRect = new Rect2I(0, 0, cols, rows);

        // 2) Морфологическая обработка, чтобы убрать щели на диагоналях
        if (BoundsGrow > 0) bm.GrowMask(BoundsGrow, fullRect);      // расширили
        if (BoundsShrink > 0) bm.GrowMask(-BoundsShrink, fullRect);   // слегка сузили обратно

        // 3) В полигоны (координаты bitmap'а -> масштабируем в мир -> в локальные)
        var polys = bm.OpaqueToPolygons(fullRect, BoundsEpsilon);
        foreach (var poly in polys)
        {
            if (poly.Length < 3) continue;

            var vertsLocal = new Vector2[poly.Length];
            for (int k = 0; k < poly.Length; k++)
            {
                var world = _gridOrigin + poly[k] * CellSize;
                vertsLocal[k] = ToLocal(world);
            }
            _boundsPolysLocal.Add(vertsLocal);
        }
    }


    // =====================================================================
    // ОТРИСОВКА



    public override void _Draw()
    {
        if (DrawTreeEdges)
        {
            foreach (var kv in _parent)
            {
                var id = kv.Key;
                var pid = kv.Value;
                if (id == pid || !_reachableIds.Contains(id)) continue;

                DrawLine(ToLocal(_idToWorld[pid]), ToLocal(_idToWorld[id]),
                         EdgeColor, EdgeWidth);
            }
        }

        if (DrawPolygonBounds)
        {
            // Сначала заливка, потом обводка
            foreach (var poly in _boundsPolysLocal)
                DrawPolygon(poly, new[] { BoundsFill });

            foreach (var poly in _boundsPolysLocal)
            {
                for (int i = 0; i < poly.Length; i++)
                    DrawLine(poly[i], poly[(i + 1) % poly.Length],
                             BoundsOutline, BoundsOutlineWidth);
            }
        }
        else
        {
            foreach (var id in _reachableIds)
            {
                DrawCircle(ToLocal(_idToWorld[id]), DotRadius, DotFill);
            }
        }

        if (_isPreviewing && _previewPath.Length > 1)
        {
            DrawArrowWorld(_previewPath);
        }

        if (_isMoving && _movePath.Length > 1)
        {
            DrawArrowWorld(_movePath);
        }
    }


    private void DrawArrowWorld(Vector2[] worldPts)
    {
        var local = new Vector2[worldPts.Length];
        for (var i = 0; i < worldPts.Length; i++) local[i] = ToLocal(worldPts[i]);

        DrawPolyline(local, PathColor, PathWidth);

        var n = local.Length;
        if (n >= 2)
        {
            var a = local[n - 2];
            var b = local[n - 1];
            var dir = (b - a).Normalized();
            var perp = new Vector2(-dir.Y, dir.X);

            var tip = b;
            var baseMid = b - dir * ArrowHeadLength;
            var p1 = baseMid + perp * ArrowHeadHalfWidth;
            var p2 = baseMid - perp * ArrowHeadHalfWidth;

            DrawPolygon(new[] { tip, p1, p2 }, new[] { PathColor });
        }
    }
}
