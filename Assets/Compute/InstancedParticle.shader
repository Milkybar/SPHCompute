Shader "Particle/Instanced"
{
    SubShader
    {
        Tags 
        { 
            "RenderType"="Opaque" 
        }

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 5.0

            #include "UnityCG.cginc"

            struct Particle
            {
                float2 m_Position;
                float2 m_PositionProj;
                float2 m_PositionLast;
                float2 m_Velocity;
                float m_Pressure;
                float m_PressureV;
            };

            StructuredBuffer<Particle> Particles;

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v, uint instanceID : SV_InstanceID)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex * 0.06 + float3(Particles[instanceID].m_Position, 0));
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return float4(0, 1, 0, 1);
            }
            ENDCG
        }
    }
}
