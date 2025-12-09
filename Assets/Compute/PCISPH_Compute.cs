using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// todo(luke) : add padding cells to remove range check for neightbouring cells
// todo(luke) : try build per particle neightbour list to speed to neighbour checks
// todo(luke) : add walls and floor colliders for more interesting simulation shapes
// todo(luke) : track a 'foam' value that increases with velocity, decreases over time

namespace _4
{
	public struct Particle
	{
		public Vector2 m_Position;
		public Vector2 m_PositionProj;
		public Vector2 m_PositionLast;
		public Vector2 m_Velocity;
		public float m_Pressure;
		public float m_PressureV;
	}

	public class PCISPH_Compute : MonoBehaviour
	{
		const int solverSteps = 10;
		const float particleRadius = 0.03f;
		const float h = 6.0f * particleRadius;
		const int px = 410;
		const int py = 120;
		const float width = 52;
		const float height = 30;


		public ComputeShader m_Shader;
		public Mesh m_InstanceMesh;
		public Material m_InstanceMaterial;

		private int CSMain_ApplyExternalForces;
		private int CSMain_Integrate;
		private int CSMain_PressureStep;
		private int CSMain_Project;
		private int CSMain_Correct;
		private int CSMain_EnforceBoundary;
		private int CSMain_GridReset;
		private int CSMain_GridCount;
		//private int CSMain_GridMaintain;

		private ComputeBuffer m_Particles;
		private ComputeBuffer m_GridCount;
		//private ComputeBuffer m_GridCount_Other;
		//private ComputeBuffer m_GridCountA;
		//private ComputeBuffer m_GridCountB;
		private ComputeBuffer m_GridLookup;
		//private ComputeBuffer m_GridLookup_Other;
		//private ComputeBuffer m_GridLookupA;
		//private ComputeBuffer m_GridLookupB;
		private int m_ParticleCount = 0;
		private ComputeBuffer m_ArgsBuffer;

		const float c_GridCellSize = 0.2f;
		const int c_MaxCellParticles = 32;
		private int _gridx;
		private int _gridy;


#if DEBUG_DRAW
		private int[] _GridCountDebug;
		private uint[] _GridLookupDebug;
		private Particle[] _ParticlesDebug;
		private Color[] _CellColour;
		private int _CellMaxCounter = 0;
#endif

		void Start()
		{
			// kernels
			CSMain_ApplyExternalForces = m_Shader.FindKernel( "CSMain_ApplyExternalForces" );
			CSMain_Integrate = m_Shader.FindKernel( "CSMain_Integrate" );
			CSMain_PressureStep = m_Shader.FindKernel( "CSMain_PressureStep" );
			CSMain_Project = m_Shader.FindKernel( "CSMain_Project" );
			CSMain_Correct = m_Shader.FindKernel( "CSMain_Correct" );
			CSMain_EnforceBoundary = m_Shader.FindKernel( "CSMain_EnforceBoundary" );
			CSMain_GridReset = m_Shader.FindKernel( "CSMain_GridReset" );
			CSMain_GridCount = m_Shader.FindKernel( "CSMain_GridCount" );
			//CSMain_GridMaintain = m_Shader.FindKernel( "CSMain_GridMaintain" );

			// particles
			m_Particles = new ComputeBuffer( px * py, sizeof( float ) * 10, ComputeBufferType.Structured );
			Particle[] particles = new Particle[px * py];

			for( int x = 0; x < px; x++ )
			{
				for( int y = 0; y < py; y++ )
				{
					Vector2 position = new Vector2(
						1.0f + x * particleRadius * 4.0f + Random.value * 0.001f,
						2.0f + y * particleRadius * 4.0f + Random.value * 0.001f );

					particles[x * py + y] = new Particle
					{
						m_Position = position,
						m_PositionProj = position,
						m_PositionLast = position,
						m_Velocity = Vector2.zero,
						m_Pressure = 0.0f,
						m_PressureV = 0.0f
					};
				}
			}

			m_Particles.SetData( particles );
			m_ParticleCount = particles.Length;

			// grid count
			_gridx = Mathf.CeilToInt( Mathf.CeilToInt( width / c_GridCellSize ) / 8.0f ) * 8;
			_gridy = Mathf.CeilToInt( Mathf.CeilToInt( height / c_GridCellSize ) / 8.0f ) * 8;
			int[] counters = new int[_gridx * _gridy];
			m_GridCount = new ComputeBuffer( _gridx * _gridy, sizeof( int ), ComputeBufferType.Raw );
			m_GridCount.SetData( counters );
			//m_GridCountB = new ComputeBuffer( _gridx * _gridy, sizeof( int ), ComputeBufferType.Structured );
			//m_GridCountB.SetData( counters );

#if DEBUG_DRAW
			_GridCountDebug = new int[_gridx * _gridy];
			_GridLookupDebug = new uint[_gridx * _gridy * c_MaxCellParticles];
			_ParticlesDebug = new Particle[px * py];
			_CellColour = new Color[_gridx * _gridy];
			for( int c = 0; c < _CellColour.Length; c++ )
			{
				_CellColour[c] = Random.ColorHSV( 0, 1, 0.5f, 1, 0.5f, 1 );
			}
#endif

			// grid lookup
			m_GridLookup = new ComputeBuffer( _gridx * _gridy * c_MaxCellParticles, sizeof( uint ), ComputeBufferType.Structured );
			//m_GridLookupB = new ComputeBuffer( _gridx * _gridy * c_MaxCellParticles, sizeof( uint ), ComputeBufferType.Structured );


			// args buffer
			uint numIndices = m_InstanceMesh.GetIndexCount( 0 );
			uint[] args = new uint[5] { numIndices, (uint)m_ParticleCount, 0, 0, 0 };
			m_ArgsBuffer = new ComputeBuffer( 1, args.Length * sizeof( uint ), ComputeBufferType.IndirectArguments );
			m_ArgsBuffer.SetData( args );


			// build grid structure
			//m_GridCount = m_GridCountA;
			//m_GridCount_Other = m_GridCountB;
			//m_GridLookup = m_GridLookupA;
			//m_GridLookup_Other = m_GridLookupB;

			m_Shader.SetBuffer( CSMain_GridReset, "GridCount", m_GridCount );
			m_Shader.Dispatch( CSMain_GridReset, _gridx / 8, _gridy / 8, 1 );

			int xGroups = Mathf.CeilToInt( m_ParticleCount / 64.0f );
			m_Shader.SetInt( "ParticleCount", m_ParticleCount );
			m_Shader.SetInt( "GridX", _gridx );
			m_Shader.SetInt( "GridY", _gridy );
			m_Shader.SetBuffer( CSMain_GridCount, "Particles", m_Particles );
			m_Shader.SetBuffer( CSMain_GridCount, "GridCount", m_GridCount );
			m_Shader.SetBuffer( CSMain_GridCount, "GridLookup", m_GridLookup );
			m_Shader.Dispatch( CSMain_GridCount, xGroups, 1, 1 );
		}

		private void OnDestroy()
		{
			m_Particles.Release();
			m_GridCount.Release();
			//m_GridCountB.Release();
			m_GridLookup.Release();
			//m_GridLookupB.Release();
			m_ArgsBuffer.Release();
		}

		void Update()
		{
			int xGroups = Mathf.CeilToInt( m_ParticleCount / 64.0f );
			m_Shader.SetInt( "ParticleCount", m_ParticleCount );
			m_Shader.SetInt( "GridX", _gridx );
			m_Shader.SetInt( "GridY", _gridy );

			m_Shader.SetFloat( "Time", Time.time * 2.0f );

			for( int i = 0; i < solverSteps; ++i )
			{
				//// todo(luke) : dont do this every step
				//// Count grids
				//m_Shader.SetBuffer( CSMain_GridCount, "Particles", m_Particles );
				//m_Shader.SetBuffer( CSMain_GridCount, "GridCount", m_GridCount );
				//m_Shader.SetBuffer( CSMain_GridCount, "GridLookup", m_GridLookup );
				//m_Shader.Dispatch( CSMain_GridCount, _gridx / 8, _gridy / 8, 1 );


#if DEBUG_DRAW
				// Get some debug data out!
				m_GridCount.GetData( _GridCountDebug );
				m_GridLookup.GetData( _GridLookupDebug );
				m_Particles.GetData( _ParticlesDebug );
				done = true;
#endif


				// Pressure step
				m_Shader.SetBuffer( CSMain_PressureStep, "Particles", m_Particles );
				m_Shader.SetBuffer( CSMain_PressureStep, "GridCount", m_GridCount );
				m_Shader.SetBuffer( CSMain_PressureStep, "GridLookup", m_GridLookup );
				m_Shader.Dispatch( CSMain_PressureStep, xGroups, 1, 1 );

				// Project
				m_Shader.SetBuffer( CSMain_Project, "Particles", m_Particles );
				m_Shader.SetBuffer( CSMain_Project, "GridCount", m_GridCount );
				m_Shader.SetBuffer( CSMain_Project, "GridLookup", m_GridLookup );
				m_Shader.Dispatch( CSMain_Project, xGroups, 1, 1 );

				// Correct
				m_Shader.SetBuffer( CSMain_Correct, "Particles", m_Particles );
				m_Shader.Dispatch( CSMain_Correct, xGroups, 1, 1 );

				// Apply external forces
				m_Shader.SetBuffer( CSMain_ApplyExternalForces, "Particles", m_Particles );
				m_Shader.Dispatch( CSMain_ApplyExternalForces, xGroups, 1, 1 );

				// Integrate
				m_Shader.SetBuffer( CSMain_Integrate, "Particles", m_Particles );
				m_Shader.Dispatch( CSMain_Integrate, xGroups, 1, 1 );

				// Enforce boundaries
				m_Shader.SetBuffer( CSMain_EnforceBoundary, "Particles", m_Particles );
				m_Shader.Dispatch( CSMain_EnforceBoundary, xGroups, 1, 1 );


				//// Maintain the grid lookup
				//m_Shader.SetBuffer( CSMain_GridMaintain, "Particles", m_Particles );
				//m_Shader.SetBuffer( CSMain_GridMaintain, "GridCount", m_GridCount );
				//m_Shader.SetBuffer( CSMain_GridMaintain, "GridLookup", m_GridLookup );
				////m_Shader.SetBuffer( CSMain_GridMaintain, "GridCount2", m_GridCount_Other );
				////m_Shader.SetBuffer( CSMain_GridMaintain, "GridLookup2", m_GridLookup_Other );
				//m_Shader.Dispatch( CSMain_GridMaintain, _gridx / 8, _gridy / 8, 1 );

				// swap the grid buffers around after tje maintaince
				//var tempCount = m_GridCount;
				//m_GridCount = m_GridCount_Other;
				//m_GridCount_Other = tempCount;
				//
				//var tempLookup = m_GridLookup;
				//m_GridLookup = m_GridLookup_Other;
				//m_GridLookup_Other = tempLookup;


				m_Shader.SetBuffer( CSMain_GridReset, "GridCount", m_GridCount );
				m_Shader.Dispatch( CSMain_GridReset, _gridx / 8, _gridy / 8, 1 );

				m_Shader.SetBuffer( CSMain_GridCount, "Particles", m_Particles );
				m_Shader.SetBuffer( CSMain_GridCount, "GridCount", m_GridCount );
				m_Shader.SetBuffer( CSMain_GridCount, "GridLookup", m_GridLookup );
				m_Shader.Dispatch( CSMain_GridCount, xGroups, 1, 1 );
			}

			// Render
			m_InstanceMaterial.SetBuffer( "Particles", m_Particles );
			Graphics.DrawMeshInstancedIndirect( m_InstanceMesh, 0, m_InstanceMaterial, new Bounds( Vector3.zero, Vector3.one * 10000.0f ), m_ArgsBuffer );
		}

		private void OnDrawGizmos()
		{
			Gizmos.color = new Color( 0, 1, 0, 0.5f );
			Gizmos.DrawLine( new Vector2( 0, 0 ), new Vector2( width, 0 ) );
			Gizmos.DrawLine( new Vector2( 0, height ), new Vector2( width, height ) );
			Gizmos.DrawLine( new Vector2( 0, 0 ), new Vector2( 0, height ) );
			Gizmos.DrawLine( new Vector2( width, 0 ), new Vector2( width, height ) );

			//float wall = ( 1.0f - Mathf.Abs( Mathf.Sin( Time.time * 2.0f ) ) ) * 5.0f;
			//Gizmos.DrawLine( new Vector2( wall, 0 ), new Vector2( wall, height ) );


#if DEBUG_DRAW
			if( _GridCountDebug == null ) return;

			/*
			// draw out the particle counter
			int countMax = 0;
			for( int i = 0; i < _GridCountDebug.Length; i++ )
			{
				countMax = Mathf.Max( countMax, _GridCountDebug[i] );

				if( _GridCountDebug[i] > 32 )
				{
					int y = i / _gridx;
					int x = i - y * _gridx;
					UnityEditor.Handles.Label( new Vector2( x, y ) * c_GridCellSize, _GridCountDebug[i].ToString() );
					Debug.Log( "Overflow!!!" );
				}
			}

			if( countMax > _CellMaxCounter )
			{
				_CellMaxCounter = countMax;
				Debug.Log( $"Grid max counter: {_CellMaxCounter}" );
			}
			*/

			if( done == false ) return;

			//Gizmos.color = Color.red;
			for( int x = 0; x < _gridx; x++ )
			{
				for( int y = 0; y < _gridy; y++ )
				{
					int cell_index = x + y * _gridx;
					Gizmos.color = _CellColour[cell_index];

					int ps = cell_index * c_MaxCellParticles;
					int pe = ps + _GridCountDebug[cell_index];

					for( int p = ps; p < pe; p++ )
					{
						uint p_index = _GridLookupDebug[p];
						var particle = _ParticlesDebug[p_index];
						Gizmos.DrawSphere( particle.m_Position, particleRadius );
					}
				}
			}
#endif
		}
	}
}