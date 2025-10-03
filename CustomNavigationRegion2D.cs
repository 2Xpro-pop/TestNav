using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TestNav;
public sealed partial class CustomNavigationRegion2D: Area2D
{
	[Export]
	public float EnterCost { get; set; } = 0f;

	[Export]
	public float TravelCost { get; set; } = 1f;
}
