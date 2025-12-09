using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

namespace _3
{
	public class MyList<T>
	{
		private int m_Count;
		private T[] m_Data;


		public MyList( int capacity )
		{
			m_Count = 0;
			m_Data = new T[capacity];
		}

		public void Add( T item )
		{
			if( m_Data == null )
			{
				m_Data = new T[16];
			}
			else if( m_Data.Length == m_Count )
			{
				T[] newData = new T[m_Data.Length * 2];
				for( int i = 0; i < m_Data.Length; i++ )
				{
					newData[i] = m_Data[i];
				}
				m_Data = newData;
			}

			m_Data[m_Count] = item;
			m_Count++;
		}

		public void Clear()
		{
			m_Count = 0;
		}

		public void Pop()
		{
			m_Count--;
		}

		public T this[int index]
		{
			get => m_Data[index];
			set => m_Data[index] = value;
		}

		public int Count => m_Count;
	}

	public struct Particle
	{
		public Vector2 position;
		public Vector2 positionProj;
		public Vector2 positionLast;
		public Vector2 velocity;
		public float pressure;
		public float pressureVary;
		//public float density;
		//public float densityVary;
	}

	public class PCISPH_2 : MonoBehaviour
	{
		//List<Particle> particles = new List<Particle>();
		Dictionary<Vector2Int, MyList<Particle>> grid = new Dictionary<Vector2Int, MyList<Particle>>();
		List<KeyValuePair<Vector2Int, MyList<Particle>>> activeCells = new List<KeyValuePair<Vector2Int, MyList<Particle>>>();
		Vector2Int[] offsets = new Vector2Int[]
		{
			new Vector2Int(-1, -1),
			new Vector2Int(+0, -1),
			new Vector2Int(+1, -1),
			new Vector2Int(-1, +0),
			new Vector2Int(+0, +0),
			new Vector2Int(+1, +0),
			new Vector2Int(-1, +1),
			new Vector2Int(+0, +1),
			new Vector2Int(+1, +1),
		};
		int m_CellMax = 0;


		const int solverSteps = 10; // can modify values to run at half the fps, nice double performance win
		const int fps = 60;// / 2;
		const float eps = 0.00001f;
		const float eps2 = eps * eps;
		const float restDensity = 35.0f;
		const float stiffness = 0.08f;// / 2.0f;
		const float stiffApprox = 0.1f;// / 2.0f;
		const float surfaceTension = 0.0001f;// * 2.0f;
		const float linearVisc = 0.25f;// * 2.0f;
		const float quadVisc = 0.5f;// * 2.0f * 2.0f;
		const float particleRadius = 0.03f;
		const float h = 6.0f * particleRadius;
		const float h2 = h * h;
		const float dt = ( 1.0f / fps ) / solverSteps;
		const float dt2 = dt * dt;
		const float kern = 20.0f / ( 2.0f * Mathf.PI * h2 );
		const float kernNorm = 30.0f / ( 2.0f * Mathf.PI * h2 );
		readonly Vector2 gravityDt = new Vector2( 0, -9.81f ) * dt;
		const float particleMass = 1.0f;


		const float width = 6.0f;
		const float height = 4.0f;
		const float cellSize = h;
		readonly int cellX = Mathf.CeilToInt( width / cellSize );
		readonly int cellY = Mathf.CeilToInt( height / cellSize );



		private void ApplyExternalForces()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					var pi = particles[i];
					pi.velocity += gravityDt;
					particles[i] = pi;
				}
			}
		}

		private void Integrate()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					var pi = particles[i];
					pi.positionLast = pi.position;
					pi.position += pi.velocity * dt;
					particles[i] = pi;
				}
			}
		}

		private void PressureStep()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					var pi = particles[i];

					float dens = 0.0f;
					float densProj = 0.0f;

					for( int o = 0; o < offsets.Length; ++o )
					{
						Vector2Int cell = pair.Key + offsets[o];
						if( grid.TryGetValue( cell, out var particles_j ) == false ) continue;

						for( int j = 0; j < particles_j.Count; ++j )
						{
							//if( pair.Key == cell && i == j ) continue;
							var pj = particles_j[j];

							Vector2 dx = pj.position - pi.position;
							float r2 = dx.sqrMagnitude;
							if( r2 < eps2 || r2 > h2 ) continue;

							float r = Mathf.Sqrt( r2 );
							float a = 1.0f - r / h;
							dens += particleMass * a * a * a * kern;
							densProj += particleMass * a * a * a * a * kernNorm;
						}
					}

					//pi.density = dens;
					//pi.densityVary = densProj;
					pi.pressure = stiffness * ( dens - particleMass * restDensity );
					pi.pressureVary = stiffApprox * densProj;

					particles[i] = pi;
				}
			}
		}

		private void Project()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					var pi = particles[i];
					Vector2 xx = pi.position;

					for( int o = 0; o < offsets.Length; ++o )
					{
						Vector2Int cell = pair.Key + offsets[o];
						if( grid.TryGetValue( cell, out var particles_j ) == false ) continue;

						for( int j = 0; j < particles_j.Count; ++j )
						{
							//if( pair.Key == cell && i == j ) continue;
							var pj = particles_j[j];

							Vector2 dx = pj.position - pi.position;
							float r2 = dx.sqrMagnitude;
							if( r2 < eps2 || r2 > h2 ) continue;

							float r = Mathf.Sqrt( r2 );
							float a = 1.0f - r / h;
							float d = dt2 * ( ( pi.pressureVary + pj.pressureVary ) * a * a * a * kernNorm + ( pi.pressure + pj.pressure ) * a * a * kern ) / 2.0f;

							// relaxation
							xx -= d * dx / ( r * particleMass );

							// surface tension
							xx += ( surfaceTension / particleMass ) * particleMass * a * a * kern * dx;
							// todo(luke) : only do this if materils are of same type
							// not sure we need the divide and multiply by pass if both particles have the same mass

							// viscosity
							Vector2 dv = pi.velocity - pj.velocity;
							float u = Vector2.Dot( dv, dx );
							if( u > 0 )
							{
								u /= r;
								float I = 0.5f * dt * a * ( linearVisc * u + quadVisc * u * u );
								xx -= I * dx * dt;
							}
						}
					}

					pi.positionProj = xx;
					particles[i] = pi;
				}
			}
		}

		private void Correct()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					var pi = particles[i];
					pi.position = pi.positionProj;
					pi.velocity = ( pi.position - pi.positionLast ) / dt; // todo(luke) : depending on when integration step is we might not need to store this???
					particles[i] = pi;
				}
			}
		}

		private void EnforceBoundary()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					var pi = particles[i];

					if( pi.position.x < 0.0f )
					{
						pi.position.x = 0.0f + Random.value * 0.001f;
						pi.velocity.x *= -0.5f;
					}

					if( pi.position.y < 0.0f )
					{
						pi.position.y = 0.0f + Random.value * 0.001f; ;
						pi.velocity.y *= -0.5f;
					}

					if( pi.position.x > width )
					{
						pi.position.x = width - Random.value * 0.001f; ;
						pi.velocity.x *= -0.5f;
					}

					if( pi.position.y > height )
					{
						pi.position.y = height - Random.value * 0.001f; ;
						pi.velocity.y *= -0.5f;
					}

					particles[i] = pi;
				}
			}
		}

		private void UpdateGrid()
		{
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					Vector2Int cell = GetCell( particles[i].position );
					if( pair.Key == cell ) continue;

					// copy into new list
					grid[cell].Add( particles[i] );

					// backswap remove from current list
					particles[i] = particles[particles.Count - 1];
					particles.Pop();
				}
			}
		}

		private Vector2Int GetCell( Vector2 position )
		{
			int x = Mathf.FloorToInt( position.x / cellSize );
			int y = Mathf.FloorToInt( position.y / cellSize );
			return new Vector2Int( Mathf.Clamp( x, 0, cellX - 1 ), Mathf.Clamp( y, 0, cellY ) );
		}


		void Start()
		{
			for( int x = 0; x < cellX; x++ )
			{
				for( int y = 0; y < cellY; y++ )
				{
					grid.Add( new Vector2Int( x, y ), new MyList<Particle>( 8 ) );
				}
			}

			for( int i = 0; i < 25; i++ )
			{
				for( int j = 0; j < 20; j++ )
				{
					Vector2 position = new Vector2(
						2.0f + i * particleRadius * 3.0f + Random.value * 0.001f,
						2.0f + j * particleRadius * 3.0f + Random.value * 0.001f );

					Particle p = new Particle
					{
						position = position,
						positionProj = position,
						positionLast = position,
						velocity = Vector2.zero
					};

					Vector2Int cell = GetCell( position );
					grid[cell].Add( p );
				}
			}
		}

		void Update()
		{
			for( int i = 0; i < solverSteps; i++ )
			{
				Profiler.BeginSample( "PressureStep" );
				PressureStep();
				Profiler.EndSample();

				Profiler.BeginSample( "Project" );
				Project();
				Profiler.EndSample();

				Profiler.BeginSample( "Correct" );
				Correct();
				Profiler.EndSample();

				Profiler.BeginSample( "ApplyExternalForces" );
				ApplyExternalForces();
				Profiler.EndSample();

				Profiler.BeginSample( "Integrate" );
				Integrate();
				Profiler.EndSample();

				Profiler.BeginSample( "EnforceBoundary" );
				EnforceBoundary();
				Profiler.EndSample();

				Profiler.BeginSample( "UpdateGrid" );
				UpdateGrid();
				Profiler.EndSample();

				Profiler.BeginSample( "UpdateActiveCells" );
				int cellMax = 0;
				activeCells.Clear();
				foreach( var pair in grid )
				{
					cellMax = Mathf.Max( cellMax, pair.Value.Count );
					if( pair.Value.Count > 0 ) activeCells.Add( pair );
				}
				Profiler.EndSample();


				if( cellMax > m_CellMax )
				{
					m_CellMax = cellMax;
					Debug.Log( $"New cell max: {m_CellMax}" );
				}
			}
		}

		private void OnDrawGizmos()
		{
			// grid
			Gizmos.color = new Color( 0, 1, 0, 0.5f );
			//for( int x = 0; x < cellX + 1; x++ )
			//{
			//	Gizmos.DrawLine( new Vector2( x * cellSize, 0 ), new Vector2( x * cellSize, cellY * cellSize ) );
			//}
			//for( int y = 0; y < cellY + 1; y++ )
			//{
			//	Gizmos.DrawLine( new Vector2( 0, y * cellSize ), new Vector2( cellX * cellSize, y * cellSize ) );
			//}
			Gizmos.DrawLine( new Vector2( 0, 0 ), new Vector2( width, 0 ) );
			Gizmos.DrawLine( new Vector2( 0, height ), new Vector2( width, height ) );
			Gizmos.DrawLine( new Vector2( 0, 0 ), new Vector2( 0, height ) );
			Gizmos.DrawLine( new Vector2( width, 0 ), new Vector2( width, height ) );

			// particles
			Gizmos.color = new Color( 0, 1, 0, 1 );
			foreach( var pair in activeCells )
			{
				var particles = pair.Value;
				for( int i = 0; i < particles.Count; ++i )
				{
					Gizmos.DrawSphere( particles[i].position, particleRadius );
				}
			}
		}
	}
}