#pragma once
#include <xmmintrin.h>

#include <d3d11.h>
#include <d3dx11.h>
#include <d3d9.h>
#include <d3dx9math.h>
#include <d3dx10.h>
#include <ddraw.h>

#pragma warning (disable:4099) //  type name first seen using 'class' now seen using 'struct'
#pragma warning (disable:4244) // '+=' : conversion from 'int' to 'WORD', possible loss of data
#pragma warning (disable:4305) // truncation from 'double' to 'float
#pragma warning (disable:4800) //forcing value to bool 'true' or 'false'
#pragma comment(lib, "Winmm.lib" )
#pragma comment(lib, "d3dx9.lib")
#pragma comment(lib,"d3dx10.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dx11.lib")

#define CONCAT_IMPL(x, y) x##y
#define MACRO_CONCAT(x, y) CONCAT_IMPL(x, y)
#define PAD(SIZE) BYTE MACRO_CONCAT(_pad, __COUNTER__)[SIZE];
#define M_PI 3.14

static bool IsValid(void* ptr);

#define OFFSET_CLIENTGAMECONTEXT		0x02380B58
#define OFFSET_DXRENDERER				0x023577D4
#define OFFSET_GAMERENDERER				0x02384D78
#define OFFSET_UPDATEMATRICES			0x006C3A90
#define OFFSET_BorderInputNode			0x02384EB8
#define OFFSET_NO_PBSS					0x0235DB14

#define OFF_DEBUGRENDERER2_DRAWTEXT		0x004B7610
#define OFF_DEBUGRENDERER2_DRAWLINE		0x004B9980
#define OFF_DEBUGRENDERER2_DRAWRECT		0x4BA4F0
#define OFF_DEBUGRENDERER2_FILLRECT		0x4BA4F0
#define OFF_DEBUGRENDERER2_SINGLETON	0x4B2EA0
#define OFF_DEBUGRENDERER2_SPHERE		0x0043AF20

namespace eastl
{
	template <class T> class Tuple2
	{
	public:
		Tuple2(T e1, T e2) : _e1(e1), _e2(e2)
		{
		}

	public:
		T _e1;
		T _e2;
	};

	template <class T, INT Count, INT PadSize>

	class fixed_vector
	{
	private:
		T* m_firstElement;
		T* m_lastElement;
		T* m_arrayBound;
		//LPVOID m_pad[PadSize];
		T m_data[Count];

	public:
		fixed_vector() {
			m_firstElement = (T *)m_data;
			m_lastElement = (T *)m_data;
			m_arrayBound = (T *)&m_data[Count];
		}

		void push_back(T *const value) {
			if (m_lastElement > m_arrayBound) {
				return;
			}
			*m_lastElement = *value;
			m_lastElement = m_lastElement + 1;
		};

		void clear() {
			m_firstElement = m_data;
			m_lastElement = m_data;
		}

		UINT Size() {
			return (((DWORD)m_lastElement - (DWORD)m_firstElement) / 4);
		}

		T At(INT nIndex) {
			return *(T*)((DWORD)m_firstElement + (nIndex * sizeof(T)));
		}

		T operator [](INT index) { return At(index); }
	};

	template <class T1, class T2>

	class pair
	{
	public:
		T1 first;
		T2 second;
	};
	template <class T>
	class vector
	{
	private:
		T* m_firstElement;
		T* m_lastElement;
		T* m_arrayBound;
		LPVOID vftable;
	public:
		UINT Size() { return (((DWORD)m_lastElement - (DWORD)m_firstElement) / 4); }
		T At(INT nIndex) { return *(T*)((DWORD)m_firstElement + (nIndex * sizeof(T))); }

		T operator [](INT index) { return At(index); }
	};

	template <class T, INT Count, INT PadSize>
	class fixed_vector3
	{
	private:
		T* m_firstElement;
		T* m_lastElement;
		T* m_arrayBound;
		LPVOID m_pad[PadSize];
		T m_data[Count];

	public:
		UINT Size() { return (((DWORD)m_lastElement - (DWORD)m_firstElement) / 4); }
		T At(INT nIndex) { return *(T*)((DWORD)m_firstElement + (nIndex * sizeof(T))); }

		T operator [](INT index) { return At(index); }
	};

	template <class T1, class T2>
	class map_node
	{
	public:
		map_node<T1, T2>* m_right;
		map_node<T1, T2>* m_left;
		map_node<T1, T2>* m_parent;
		T1 value1;
		T2 value2;
	};

	template <class T1, class T2>
	class map
	{
	public:
		UINT m_Compare;
		map_node<T1, T2>* m_anchorLeft;
		map_node<T1, T2>* m_anchorRight;
		DWORD pad;
		UINT m_size;
		DWORD m_allocator;
	};

	template <class T>
	class DequeIterator
	{
	public:
		T** m_current;
		T** m_begin;
		T** m_end;
		PAD(0x4);
	};

	template <class T>
	class deque
	{
	public:
		T*** m_array;
		UINT m_count;
		DequeIterator<T> m_begin;
		DequeIterator<T> m_end;
		DWORD allocator;
	};

	template <class T>
	class basic_string
	{
	private:
		T* m_firstChar;
		T* m_lastChar;
		T* m_bufferBound;
		LPVOID vftable;

	public:
		T* GetString() { return m_firstChar; };
		operator T*(){ return m_firstChar; };
	};
};

namespace fb
{
	//template< class T > class ScopedPtr;
	namespace network
	{
		class Ghost
		{
		public:
			LPVOID vftable;				// 0x00
			PAD(0x3C);		// 0x04
		}; // 0x40

		class ClientGhost
			: public Ghost		// 0x00
		{
		}; // 0x40

		class InterpolationObject
		{
		public:
			PAD(0x14);
		}; // 0x14

		template <class T>
		class Interpolator
			: public InterpolationObject		// 0x00
		{
		public:
			eastl::deque<T> m_states;			// 0x14
		}; // 0x40

		class INetworkable
		{
		public:
			LPVOID vftable;						// 0x00
			DWORD m_networkableDescriptorSize;	// 0x04
		}; // 0x08

		class IClientNetworkable
			: public INetworkable		// 0x00
		{
		}; // 0x08

		class IClientNetworkableGroupMember
			: public IClientNetworkable		// 0x00
		{
		}; // 0x08

		class ClientNetworkableGroup
			: public IClientNetworkable
		{
		public:
			eastl::fixed_vector<IClientNetworkableGroupMember*, 32, 1> m_networkables;	// 0x00
		}; // 0x98
	};

	template <class T>
	class WeakPtr
	{
	private:
		T** m_ptr;

	public:
		T* GetData()
		{
			if (m_ptr == NULL)
				return NULL;

			//if (*m_ptr == NULL) // I often crash here .-.
			//	return NULL;

			return (T*)((DWORD)(*m_ptr) - offsetof(T, m_weakTokenHolder));
		}
	};

	template <class T>
	class Array
	{
	private:
		T* m_firstElement;

	public:
		T At(INT nIndex)
		{
			if (m_firstElement == NULL)
				return NULL;

			return *(T*)((DWORD)m_firstElement + (nIndex * sizeof(T)));
		};

		T operator [](INT index) { return At(index); }
	};

	template <class T>
	class RefArray
	{
	private:
		T** m_array;

	public:
		T* At(INT nIndex)
		{
			if (m_array == NULL)
				return NULL;

			return *(T**)((DWORD)m_array + (nIndex * 4));
		}

		T* operator [](INT index) { return At(index); }
	};

	template <class T>
	class RelocArray
	{
	private:
		UINT m_count;
		T* m_data;

	public:
		UINT Size() { return m_count; }
		T At(INT index) { return *(T*)((DWORD)m_data + (index * sizeof(T))); }
		T operator[](INT index) { return At(index); }
	};

	class MemoryArena
	{
	public:
		LPVOID vftable;
		INT m_flags;
		MemoryArena* m_front;
	};

	class String
	{
	private:
		LPSTR m_string;

	public:
		LPSTR GetString() { return m_string; }
		operator LPSTR() { return m_string; }
	};

	class Vec2
	{
	public:
		union
		{
			struct
			{
				FLOAT x;
				FLOAT y;
			};

			FLOAT data[2];
		};
	};

	class Vec3
	{
	public:
		float x, y, z, pad;

		Vec3(){}

		Vec3(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		Vec3 & operator = (Vec3 & From)
		{
			this->x = From.x;
			this->y = From.y;
			this->z = From.z;

			return (*this);
		}

		float Dot(Vec3 Vec)
		{
			return this->x * Vec.x + this->y * Vec.y + this->z * Vec.z;
		}

		float DistanceFrom2(Vec3 p)
		{
			float xD = p.x - x;
			float yD = p.y - y;
			float zD = p.z - z;

			float dist = sqrt(xD * xD + yD * yD + zD * zD);

			return dist;
		}

		float DistanceFrom(Vec3 & point)
		{
			float x2 = (point.x - x) * (point.x - x);
			float y2 = (point.y - y) * (point.y - y);
			float z2 = (point.z - z) * (point.z - z);

			float dist = sqrt(x2 + y2 + z2);

			return dist;
		}

		float Length()
		{
			float flLengthX, flLengthY, flLengthZ, flLength;

			flLengthX = x * x;
			flLengthY = y * y;
			flLengthZ = z * z;

			flLength = sqrt(flLengthX + flLengthY + flLengthZ);

			return fabs(flLength);
		}

		float Length2D()
		{
			float flLengthX, flLengthY, flLength;

			flLengthX = x * x;
			flLengthY = y * y;

			flLength = sqrt(flLengthX + flLengthY);

			return flLength;
		}

		double DotProductEx(Vec3 vec)
		{
			double temp = 0;

			temp = this->x*vec.x + this->y*vec.y + this->z*vec.z;

			return temp;
		}

		void Normalize()
		{
			float length, ilength;

			length = this->Length();

			if (length)
			{
				ilength = 1 / length;

				this->x *= ilength;
				this->y *= ilength;
				this->z *= ilength;
			}
		}

		fb::Vec3& operator + (const fb::Vec3& v) const
		{
			return fb::Vec3(x + v.x, y + v.y, z + v.z);
		}

		fb::Vec3& operator / (const float& div) const
		{
			return fb::Vec3(x / div, y / div, z / div);
		}

		fb::Vec3& operator - (const fb::Vec3& v) const
		{
			return fb::Vec3(x - v.x, y - v.y, z - v.z);
		}

		fb::Vec3& operator * (const float& mul) const
		{
			return fb::Vec3(x * mul, y * mul, z * mul);
		};//Size=0x0010
	};

	namespace physics
	{
		enum RayCastFlags
		{
			CheckDetailMesh = 0x1,
			IsAsyncRaycast = 0x2,
			DontCheckWater = 0x4,
			DontCheckTerrain = 0x8,
			DontCheckRagdoll = 0x10,
			DontCheckCharacter = 0x20,
			DontCheckGroup = 0x40,
			DontCheckPhantoms = 0x80,
		};

		class RayCastHit
		{
		public:
			fb::Vec3 m_position; //0x0000
			fb::Vec3 m_normal; //0x0010
			void* m_rigidBody; //0x0020  PhysicsEntityBase
			void* m_material; //0x0024  MaterialContainerPair
			unsigned int m_part; //0x0028
			unsigned int m_bone; //0x002C
			float m_lambda; //0x0030
		};//Size=0x0040

		class IPhysicsRayCaster
		{
		public:
			virtual bool physicsRayQuery(char* text, fb::Vec3& from, fb::Vec3& to, fb::physics::RayCastHit& hit, int flag, void* PhysicsEntityList); //
			virtual void* asyncPhysicsRayQuery(const char* ident, fb::Vec3& from, fb::Vec3& to, unsigned int flags, void* excluded);

		};//Size=0x0004

		class PhysicsRayCaster : public IPhysicsRayCaster
		{
		public:
		};
	};

	template< class T > class CtrRefBase
	{
	public:
		T* m_ptr;
	};
	template< typename T > class ScopedPtr
	{
	public:
		T* m_ptr; //0x0000
	};
	class LinearTransform
	{
	public:
		union
		{
			struct
			{
				Vec3 left;
				Vec3 up;
				Vec3 forward;
				Vec3 trans;
			};

			FLOAT data[4][4];
		};
	};
	class Vec4
	{
	public:
		float                    x, y, z, w;

		Vec4()
		{
		}

		Vec4(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		Vec4 & operator = (Vec4 & From)
		{
			this->x = From.x;
			this->y = From.y;
			this->z = From.z;
			this->w = From.w;

			return (*this);
		}

		float operator [] (int idx)
		{
			if (idx < 0 || idx > 3)
				return 0.0f;

			if (idx == 0)
				return x;
			else if (idx == 1)
				return y;
			else if (idx == 2)
				return z;
			else if (idx == 3)
				return w;

			return 0.0f;
		}

		float Dot(Vec4 Vec)
		{
			return this->x * Vec.x + this->y * Vec.y + this->z * Vec.z + this->w * Vec.w;
		}

		float Dott(Vec3 Vec)
		{
			return this->x * Vec.x + this->y * Vec.y + this->z * Vec.z;
		}

		float DistanceFrom(Vec4 Vec)
		{
			Vec4 Distance(
				Vec.x - this->x,
				Vec.y - this->y,
				Vec.z - this->z,
				Vec.w - this->w);

			return sqrt((Distance.x * Distance.x) + (Distance.y * Distance.y) + (Distance.z * Distance.z) + (Distance.w * Distance.w));
		};
	};
	class Mat4
	{
	public:
		Vec4 m_rows[4];

		Mat4()
		{
		}

		Mat4& operator = (Mat4& From)
		{
			this->m_rows[0] = From.m_rows[0];
			this->m_rows[1] = From.m_rows[1];
			this->m_rows[2] = From.m_rows[2];
			this->m_rows[3] = From.m_rows[3];

			return (*this);
		}

		Vec4& operator [] (int idx)
		{
			if (idx < 0 || idx > 3)
				return fb::Vec4();

			return m_rows[idx];
		}

		Vec4 one()
		{
			return m_rows[0];
		}

		Vec4 two()
		{
			return m_rows[1];
		}

		Vec4 three()
		{
			return m_rows[2];
		}

		Vec4 four()
		{
			return m_rows[3];
		}
	};//Size=0x0040
};

typedef D3DXVECTOR4 hkVector4;

template <class T>

class hkArray
{
public:
	T** m_data;
	INT m_size;
	INT m_capacityAndFlags;
};

class hkBaseObject
{
public:
	LPVOID vftable;		// 0x00
}; // 0x04

class hkReferencedObject
	: public hkBaseObject		// 0x00
{
public:
	WORD m_memSizeAndFlags;		// 0x04
	SHORT m_referencedCount;	// 0x06
}; // 0x08

class hkpEntityListener
{
public:
	LPVOID vftable;		// 0x00
}; // 0x04

class hkpPhantomListener
{
public:
	LPVOID vftable;		// 0x00
}; // 0x04

class hkpCharacterProxy
	: public hkReferencedObject,					// 0x00
	public hkpEntityListener,						// 0x08
	public hkpPhantomListener						// 0x0C
{
public:
	hkArray<DWORD> m_manifold;						// 0x10 hkpRootCdPoint
	hkArray<DWORD> m_bodies;						// 0x1C hkpRigidBody*
	hkArray<DWORD> m_phantoms;						// 0x28 hkpPhantom*
	hkArray<DWORD> m_overlappingTriggerVolumes;		// 0x34 hkpTriggerVolume*
	hkVector4 m_velocity;							// 0x40 //Playerspeed
	hkVector4 m_oldDisplacement;					// 0x50
	DWORD m_shapePhantom;							// 0x60
	FLOAT m_dynamicFriction;						// 0x64
	FLOAT m_staticFriction;							// 0x68
	PAD(0x4);										// 0x6C
	hkVector4 m_up;									// 0x70
	FLOAT m_extraUpStaticFriction;					// 0x80
	FLOAT m_extraDownStaticFriction;				// 0x84
	FLOAT m_keepDistance;							// 0x88
	FLOAT m_keepContactTolerence;					// 0x8C
	FLOAT m_contactAngleSensitivity;				// 0x90
	INT m_userPlanes;								// 0x94
	FLOAT m_maxCharacterSpeedForSolver;				// 0x98
	FLOAT m_characterStrength;						// 0x9C
	FLOAT m_characterMass;							// 0xA0
	hkArray<DWORD> m_listeners;						// 0xA4 hkpCharacterProxyListener*
	FLOAT m_maxSlopeCosine;							// 0xB0
	FLOAT m_penetrationRecoverySpeed;				// 0xB4
	INT m_maxCastIterations;						// 0xB8
	INT m_refreshManifoldInCheckSupport;			// 0xBC
}; // 0xC0

class hkpCharacterContext
	: public hkReferencedObject			// 0x00
{
public:
	INT m_characterType;				// 0x08
	DWORD m_stateManager;				// 0x0C hkpCharacterStateManager
	INT m_currentState;					// 0x10
	INT m_filterEnable;					// 0x14
	FLOAT m_maxLinearAcceleration;		// 0x18
	FLOAT m_maxLinearVelocity;			// 0x1C
	FLOAT m_gain;						// 0x20
}; // 0x24

class hkpShape
	: public hkReferencedObject		// 0x00
{
public:
	DWORD m_userData;				// 0x08
	INT m_type;						// 0x0C
}; // 0x10


namespace fb
{
	class AimAssist;
	class AimAssist2;
	class AimerModifierData;
	class AimingConstraints;
	class AimingPoseData;
	class AmmoConfigData;
	class AmmunitionDepot;
	class AnimatedSoldierWeapon;
	class AnimatedSoldierWeaponOffsetModule;
	class AnimatedSoldierWeaponShootModule;
	class AnimatedSoldierWeaponSpeedModule;
	class AnimatedSoldierWeaponSprintModule;
	class AnimatedSoldierWeaponZoomModule;
	class AnimatedWeaponGS;
	class AnimationConfigurationShootModuleData;
	class Asset;
	class AutoAimData;
	class AxisAlignedBox;
	class BitArray;
	class Blueprint;
	class BoltActionData;
	class BoneCollisionComponent;
	class BoneCollisionComponentData;
	class BoneCollisionData;
	class BorderInputNode;
	class BreathControlData;
	class Camera;
	class CameraContext;
	class CameraData;
	class CameraManager;
	class CameraScene;
	class CharacterEntity;
	class CharacterEntityData;
	class CharacterPhysicsData;
	class CharacterPhysicsEntity;
	class CharacterPhysicsEntityCallbacks;
	class CharacterPhysicsEntityCollisionShapes;
	class CharacterPhysicsEntityContext;
	class CharacterPhysicsEntityState;
	class CharacterPoseConstraints;
	class CharacterPoseData;
	class CharacterStateData;
	class CharacterStatePoseInfo;
	class ChassisComponent;
	class ClientAimingReplication;
	class ClientAnimatedSoldierWeaponHandler;
	class ClientBoneCollisionComponent;
	class ClientCameraContext;
	class ClientCharacterEntity;
	class ClientChassisComponent;
	class ClientChassisComponentSimulation;
	class ClientChassisComponentReplication;
	class ClientChassisComponentPrediction;
	class ClientComponent;
	class ClientControllableEntity;
	class ClientEntryComponent;
	class ClientGameContext;
	class ClientGameEntity;
	class ClientGameView;
	class ClientGhostAndNetworkableGameEntity;
	class ClientHealthStateEntityManager;
	class ClientLockingController;
	class ClientPartComponent;
	class ClientPhysicsEntity;
	class ClientPlayer;
	class ClientPlayer2;
	class ClientPlayerManager;
	class ClientPlayerManagerPlayer;
	class ClientPlayerView;
	class ClientSoldierAimingSimulation;
	class ClientSoldierEntity;
	class ClientSoldierPrediction;
	class ClientSoldierReplication;
	class ClientSoldierSimulation;
	class ClientSoldierWeapon;
	class ClientSoldierWeaponsComponent;
	class ClientSpawnEntity;
	class ClientSubView;
	class ClientVehicleEntity;
	class ClientVehicleEntityHealth;
	class ClientWeapon;
	class ClientWeaponFiringReplication;
	class ClientWeaponsState;
	class Component;
	class ComponentCollection;
	class ComponentData;
	class ControllableEntity;
	class ControllableEntityData;
	class ControllableFinder;
	class DataBusData;
	class DataContainer;
	class DxRenderer;
	class DynamicBitSet;
	class Entity;
	class EntityBus;
	class EntityBusData;
	class EntityBusPeer;
	class EntityCollectionSegment;
	class EntityCreator;
	class EntityData;
	class EntityWorld;
	class EntryComponent;
	class EntryInput;
	class EntryInputActionMap;
	class EntryInputState;
	class EntryInputTranslator;
	class EventConnection;
	class EyePositionCallback;
	class FireEffectData;
	class FireLogicData;
	class FiringDispersion;
	class FiringDispersionData;
	class FiringFunctionData;
	class FovEffect;
	class FreeCamera;
	class FreeCameraInput;
	class GameContext;
	class GameDataContainer;
	class GameEntity;

	class DebrisManager;
	class EmitterManager;
	class VegetationManager;
	class VegetationSystemSettings;

	class GameEntityData;
	class GameObjectData;
	class GamePhysicsEntityData;
	class GameRenderer;
	class GameRenderViewParams;
	class GameTime;
	class GameView;
	class GameWorld;
	class HavokAsset;
	class HealthStateEntityManager;
	class HoldAndReleaseData;
	class IClientNetworkableGroupMember;
	class IClientSoldierHealthModule;
	class IGameRenderer;
	class IInputFilter;
	class Input;
	class InputAction;
	class InputActionMap;
	class InputActionMapping;
	class InputActionMappingsData;
	class InputActions;
	class InputCache;
	class InputNode;
	class IPhysicsRayCaster;
	class IRigidBodyHook;
	class ITypedObject;
	class ITypedObjectWithRefCount;
	class Level;
	class levell;
	class LevelData;
	class LevelDescription;
	class LevelSetup;
	class LevelSetupOption;
	class LinkConnection;
	class LockingController;
	class LockingControllerData;
	class LookConstraintsData;
	class MaterialContainerPair;
	class MaterialGridData;
	class MaterialGridManager;
	class MaterialInteractionGridRow;
	class MaterialRelationPropertyPair;
	class MemoryArena;
	class MessageListener;
	class ObjectBlueprint;
	class OnlineId;
	class OverHeatData;
	class PartComponentData;
	class PathfindingBlob;
	class PhysicsEntity;
	class PhysicsEntityBase;
	class PhysicsEntityData;
	class PhysicsEntityParts;
	class PhysicsEntityUserData;
	class PitchModifier;
	class Player;
	class PlayerData;
	class PlayerManager;
	class PrefabBlueprint;
	class ProjectileBlueprint;
	class ProjectileEntityData;
	class PropertyConnection;
	class PropertyModificationListener;
	class RayCastHit;
	class RecoilData;
	class ReferenceObjectData;
	class RefillableAmmunitionDepot;
	class RenderScreenInfo;
	class RenderView;
	class RenderViewDesc;
	class ShotConfigData;
	class SkeletonAsset;
	class SkeletonCollisionData;
	class SoldierAimAssistData;
	class SoldierAimingEnvironment;
	class SoldierAimingSimulationData;
	class SoldierEntity;
	class SoldierEntityData;
	class SoldierWeaponDispersion;
	class SoldierWeaponsComponent;
	class SpatialEntity;
	class SpatialEntityData;
	class SpatialPrefabBlueprint;
	class SpawnReferenceObjectData;
	class SpeedModifierData;
	class SubLevel;
	class SubView;
	class SubWorldData;
	class SupportedShootingCallback;
	class TargetCameraCallback;
	class TeamEntityData;
	class TeamInfo;
	class Tool;
	class ToolData;
	class VehicleEntity;
	class VehicleEntityHealth;
	class VehicleHealthZoneData;
	class Weapon;
	class WeaponAimingConfigurationModifier;
	class WeaponAimingSimulationModifier;
	class WeaponAnimTypeModifier;
	class WeaponData;
	class WeaponFiring;
	class WeaponFiringCallbackHandler;
	class WeaponFiringCallbacks;
	class WeaponFiringData;
	class WeaponFiringDataModifier;
	class WeaponFiringEffectsModifier;
	class WeaponFiringShooter;
	class WeaponMagazineModifier;
	class WeaponMiscModifierSettings;
	class WeaponModifier;
	class WeaponModifierBase;
	class WeaponOffsetData;
	class WeaponProjectileModifier;
	class WeaponShotModifier;
	class WeaponSoundModifier;
	class WeaponSpeedData;
	class WeaponsState;
	class WeaponSuppressionData;
	class WeaponSway;
	class WeaponSwayCallbackImpl;
	class WeaponSwitching;
	class WeaponSwitchingCallbacks;
	class WeaponSwitchingState;
	class WeaponZoomModifier;
	class WorldData;
	class ZoomLevelData;
	class ZoomLevelLockData;
	class ClientSoldierWeapon2;
	class ClientAntAnimatableComponent;
	class GameAnimatable;
	class HavokPhysicsManager;
	class Level2;
	class StanceFilterComponentData;
	class ClientStanceFilterComponent;
	class StanceFilterComponent;
	class StanceFilterComponentData;
	class ClientWeaponComponent;
	class WeaponInfo;
	struct RayCastHit2;
	struct PlayerScore;
	class ClientPlayerScore;
	class ClientPlayerScoreManager;
	class TypeInfoData;
	class TypeInfo;
	class ClientGrenadeEntity;
	class ClientExplosionPackEntity;
	class SoldierWeaponBlueprint;
	class PickupEntity;
	class ClientPickupEntity;
	class EntityCollection;
	class MyEntityMatrix;
	class MyEntity;
	class ClientSpottingTargetComponent;
	class BreathControlData;

	/*class UIHud
	{

		class UIEngine::UIEngineDrawComp // Inherited class at offset 0x0
		{

			virtual void * __vecDelDtor(unsigned int);      // V: 0x0

		}; // fb::UIEngine::UIEngineDrawComp

		class HudRenderState
		{

			struct fb::UIVertex2d * vertices;                     // this+0x0
			unsigned short * indices;                     // this+0x4
			unsigned int vertexBufferOffset;                     // this+0x8
			unsigned int indexBufferOffset;                     // this+0xC
			unsigned int count;                     // this+0x10
			unsigned int lastBufferSize;                     // this+0x14
			float alpha;                     // this+0x18
			function  * cb;                     // this+0x1C

		}; // HudRenderState

		class UIHudDraw
		{

			virtual void * __vecDelDtor(unsigned int);      // V: 0x0

		}; // UIHudDraw

		enum eHudRenderState
		{

			int eHudRenderState_alpha;                     // constant 0x0
			int eHudRenderState_add;                     // constant 0x1
			int eHudRenderState_count;                     // constant 0x2
			int eHudRenderState_notUsed;                     // constant 0x3

		}; // eHudRenderState

		class UIHudIconDrawParams
		{

			struct fb::Vec2 pos;                     // this+0x0
			struct fb::Vec2 size;                     // this+0x8
			struct fb::Vec2 flip;                     // this+0x10
			enum fb::UIHudIcon icon;                     // this+0x18
			enum fb::UIIconState state;                     // this+0x1C
			float scale;                     // this+0x20
			struct fb::Color32 color;                     // this+0x24

		}; // UIHudIconDrawParams

		class Text
		{

			unsigned int hash;                     // this+0x0
			float timeSinceUsed;                     // this+0x4
			float textSize;                     // this+0x8
			float glowSize;                     // this+0xC
			struct fb::Color32 color;                     // this+0x10
			struct fb::Vec2 pos;                     // this+0x14
			unsigned char justCreated;                     // this+0x1C
			unsigned char textLen;                     // this+0x1D
			unsigned short pad;                     // this+0x1E
			class GRect<float> textRect;                     // this+0x20
			class GPtr<GFxDrawText> text;                     // this+0x30
			struct GFxDrawText::Filter filter;                     // this+0x34

		}; // Text

		struct fb::Vec4 m_currentTextColor;                     // this+0x10
		bool m_bIsMultiplayer;                     // this+0x20
		bool m_bIsCoop;                     // this+0x21
		bool m_bCreateNewText;                     // this+0x22
		class fb::UIMinimapIconTextureAtlas m_atlas;                     // this+0x24
		enum fb::UIHud::eHudRenderState m_currRenderState;                     // this+0x50
		struct fb::UIHud::HudRenderState[0x2] m_renderState;                     // this+0x54
		class eastl::vector<fb::UIHud::Text *, fb::eastl_arena_allocator> m_text;                     // this+0x94
		class eastl::vector<fb::UIHud::UIHudDraw *, fb::eastl_arena_allocator> m_drawObjects;                     // this+0xA4
		float m_lastDelta;                     // this+0xB4
		virtual void * __vecDelDtor(unsigned int);      // V: 0x0

	}; // fb::UIHud*/

	/*
	struct PlayerScore
	{
	__int32 m_rank;
	__int32 m_kills;
	__int32 m_deaths;
	__int32 m_score;
	__int32 m_globalScoreOrginal;        // this+0x10
	__int32 m_globalScoreUpdated;        // this+0x14
	__int32 m_veteran;                   // this+0x18
	float m_time;                    // this+0x1C
	};

	class ClientPlayerScore
	{
	public:
	char pad[0x12C];
	PlayerScore m_score;
	};

	class ClientPlayerScoreManager
	{
	public:
	ClientPlayerScore * getPlayerScore(ClientPlayer * player)
	{
	DWORD dwPlayerScore = 0xB324B0;
	ClientPlayerScore * tmpPlayer;
	__asm
	{
	push player
	mov eax, dwPlayerScore
	call eax
	mov tmpPlayer, eax
	}
	return tmpPlayer;
	}
	};
	*/

	/*class fb::UIHud
	{

		class fb::UIEngine::UIEngineDrawComp // Inherited class at offset 0x0
		{

			virtual void * __vecDelDtor(unsigned int);      // V: 0x0

		}; // fb::UIEngine::UIEngineDrawComp

		class HudRenderState
		{

			struct fb::UIVertex2d * vertices;                     // this+0x0
			unsigned short * indices;                     // this+0x4
			unsigned int vertexBufferOffset;                     // this+0x8
			unsigned int indexBufferOffset;                     // this+0xC
			unsigned int count;                     // this+0x10
			unsigned int lastBufferSize;                     // this+0x14
			float alpha;                     // this+0x18
			function  * cb;                     // this+0x1C

		}; // HudRenderState

		class UIHudDraw
		{

			virtual void * __vecDelDtor(unsigned int);      // V: 0x0

		}; // UIHudDraw

		enum eHudRenderState
		{

			int eHudRenderState_alpha;                     // constant 0x0
			int eHudRenderState_add;                     // constant 0x1
			int eHudRenderState_count;                     // constant 0x2
			int eHudRenderState_notUsed;                     // constant 0x3

		}; // eHudRenderState

		class UIHudIconDrawParams
		{

			struct fb::Vec2 pos;                     // this+0x0
			struct fb::Vec2 size;                     // this+0x8
			struct fb::Vec2 flip;                     // this+0x10
			enum fb::UIHudIcon icon;                     // this+0x18
			enum fb::UIIconState state;                     // this+0x1C
			float scale;                     // this+0x20
			struct fb::Color32 color;                     // this+0x24

		}; // UIHudIconDrawParams

		class Text
		{

			unsigned int hash;                     // this+0x0
			float timeSinceUsed;                     // this+0x4
			float textSize;                     // this+0x8
			float glowSize;                     // this+0xC
			struct fb::Color32 color;                     // this+0x10
			struct fb::Vec2 pos;                     // this+0x14
			unsigned char justCreated;                     // this+0x1C
			unsigned char textLen;                     // this+0x1D
			unsigned short pad;                     // this+0x1E
			class GRect<float> textRect;                     // this+0x20
			class GPtr<GFxDrawText> text;                     // this+0x30
			struct GFxDrawText::Filter filter;                     // this+0x34

		}; // Text

		struct fb::Vec4 m_currentTextColor;                     // this+0x10
		bool m_bIsMultiplayer;                     // this+0x20
		bool m_bIsCoop;                     // this+0x21
		bool m_bCreateNewText;                     // this+0x22
		class fb::UIMinimapIconTextureAtlas m_atlas;                     // this+0x24
		enum fb::UIHud::eHudRenderState m_currRenderState;                     // this+0x50
		struct fb::UIHud::HudRenderState[0x2] m_renderState;                     // this+0x54
		class eastl::vector<fb::UIHud::Text *, fb::eastl_arena_allocator> m_text;                     // this+0x94
		class eastl::vector<fb::UIHud::UIHudDraw *, fb::eastl_arena_allocator> m_drawObjects;                     // this+0xA4
		float m_lastDelta;                     // this+0xB4
		virtual void * __vecDelDtor(unsigned int);      // V: 0x0

	}; // fb::UIHud*/

	class Color32
	{
	public:
		Color32(unsigned long color)
		{
			this->color = color;
		}

		Color32(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha) :
			r(red), g(green), b(blue), a(alpha)
		{
		}

		static Color32 White()
		{
			return Color32(255, 255, 255, 255);
		}

		static Color32 Black()
		{
			return Color32(0, 0, 0, 255);
		}

		static Color32 Red()
		{
			return Color32(255, 0, 0, 255);
		}

		static Color32 Green()
		{
			return Color32(0, 255, 0, 255);
		}

		static Color32 Blue()
		{
			return Color32(0, 0, 255, 255);
		}

	public:
		union
		{
			struct
			{
				unsigned char r;
				unsigned char g;
				unsigned char b;
				unsigned char a;
			};

			unsigned long color;
		};
	};

	class BreathControlData
	{
	public:
	};

	class BreathControlHandler
	{
	public:
		BreathControlData* m_data; //0x0000
		float m_breathControlTimer; //0x0004
		float m_breathControlMultiplier; //0x0008
		float m_breathControlPenaltyTimer; //0x000C
		float m_breathControlPenaltyMultiplier; //0x0010
		bool m_breathControlActive; //0x0014
		bool m_breathControlInputDown; //0x0015
	};
	enum Realm
	{
		Realm_Client,
		Realm_Server,
		Realm_ClientAndServer,
		Realm_None,
		Realm_Pipeline,
	};

	enum ResourceCompartment
	{
		ResourceCompartment_First_,
		ResourceCompartment_Static,
		ResourceCompartment_Frontend,
		ResourceCompartment_LoadingScreen,
		ResourceCompartment_Game,
		ResourceCompartment_HUD,
		ResourceCompartment_Debug,
		ResourceCompartment_Dynamic_Begin_,
		ResourceCompartment_Dynamic_Synchronized_Begin_,
		ResourceCompartment_Dynamic_Synchronized_End_,
		ResourceCompartment_Dynamic_ClientOnly_Begin_,
		ResourceCompartment_Dynamic_ClientOnly_End_,
		ResourceCompartment_Dynamic_End_,
		ResourceCompartment_Count_,
		ResourceCompartment_Forbidden,
	};
	enum InputActionMapSlot
	{
		InputActionMapSlot_Undefined,
		InputActionMapSlot_Root1,
		InputActionMapSlot_Root2,
		InputActionMapSlot_Root3,
		InputActionMapSlot_Root4,
		InputActionMapSlot_Root5,
		InputActionMapSlot_Root6,
		InputActionMapSlot_Root7,
		InputActionMapSlot_Root8,
		InputActionMapSlot_Root9,
		InputActionMapSlot_Root10,
		InputActionMapSlot_Root11,
		InputActionMapSlot_Root12,
		InputActionMapSlot_Root13,
		InputActionMapSlot_Root14,
		InputActionMapSlot_Root15,
		InputActionMapSlot_Root16,
		InputActionMapSlot_Sticks1,
		InputActionMapSlot_Sticks2,
		InputActionMapSlot_Sticks3,
		InputActionMapSlot_Sticks4,
		InputActionMapSlot_Buttons1,
		InputActionMapSlot_Buttons2,
		InputActionMapSlot_Buttons3,
		InputActionMapSlot_Buttons4,
		InputActionMapSlot_Sticks1Buttons1,
		InputActionMapSlot_Sticks1Buttons2,
		InputActionMapSlot_Sticks1Buttons3,
		InputActionMapSlot_Sticks1Buttons4,
		InputActionMapSlot_Sticks2Buttons1,
		InputActionMapSlot_Sticks2Buttons2,
		InputActionMapSlot_Sticks2Buttons3,
		InputActionMapSlot_Sticks2Buttons4,
		InputActionMapSlot_Sticks3Buttons1,
		InputActionMapSlot_Sticks3Buttons2,
		InputActionMapSlot_Sticks3Buttons3,
		InputActionMapSlot_Sticks3Buttons4,
		InputActionMapSlot_Sticks4Buttons1,
		InputActionMapSlot_Sticks4Buttons2,
		InputActionMapSlot_Sticks4Buttons3,
		InputActionMapSlot_Sticks4Buttons4,
		InputActionMapSlot_Count,
	};

	enum CameraIds
	{
		NoCameraId,
		FreeCameraId,
		EntryCameraId,
		CameraIdCount,
	};

	enum StreamRealm
	{
		StreamRealm_None,
		StreamRealm_Client,
		StreamRealm_Both,
	};

	enum PersonViewEnum
	{
		FirstPerson,
		ThirdPerson,
		PersonViewCount,
	};

	enum PlayerSpawnType
	{
		PlayerSpawnType_HumanPlayer,
		PlayerSpawnType_AiPlayer,
		PlayerSpawnType_Actor,
	};

	enum PersonViewMode
	{
		PersonViewMode_FirstPerson,
		PersonViewMode_ThirdPerson,
	};

	enum CharacterPoseType
	{
		CharacterPoseType_Stand,
		CharacterPoseType_Crouch,
		CharacterPoseType_Prone,
		CharacterPoseTypeCount,
	};

	enum CharacterPoseCollisionType
	{
		CharacterPoseCollisionType_Capsule,
		CharacterPoseCollisionType_Pencil,
	};

	enum CharacterStateType
	{
		CharacterStateType_OnGround,
		CharacterStateType_Jumping,
		CharacterStateType_InAir,
		CharacterStateType_Climbing,
		CharacterStateType_Falling,
		CharacterStateType_User_0,
		CharacterStateType_User_1,
		CharacterStateType_User_2,
		CharacterStateType_User_3,
		CharacterStateType_User_4,
		CharacterStateType_User_5,
		CharacterStateType_StateCount,
		CharacterStateType_Parachute,
		CharacterStateType_Swimming,
		CharacterStateType_AnimationControlled,
		CharacterStateType_Sliding,
	};

	enum SoldierEntityActionState
	{
		Jumping,
		Walking,
		Sliding,
		Air,
		Falling,
		Parachute,
		Swim,
		Climb,
		AnimationDriven,
		NumberOfBits,
	};

	enum HitReactionType
	{
		HRT_Body,
		HRT_Head,
		HRT_RightArm,
		HRT_LeftArm,
		HRT_RightLeg,
		HRT_LeftLeg,
		HRT_Count,
	};

	enum EntryInputActionEnum
	{
		EIAThrottle,
		EIAStrafe,
		EIABrake,
		EIASwitchPrimaryInventory,
		EIAYaw,
		EIAPitch,
		EIARoll,
		EIAFire,
		EIACameraPitch,
		EIACameraYaw,
		EIAFireCountermeasure,
		EIAZoom,
		EIAJump,
		EIAChangePose,
		EIAProne,
		EIAReload,
		EIASelectWeapon1,
		EIASelectWeapon2,
		EIASelectWeapon3,
		EIASelectWeapon4,
		EIASelectWeapon5,
		EIASelectWeapon6,
		EIASelectWeapon7,
		EIASelectWeapon8,
		EIASelectWeapon9,
		EIASwitchPrimaryWeapon,
		EIAGrenadeLauncher,
		EIAStaticGadget,
		EIADynamicGadget1,
		EIADynamicGadget2,
		EIAMeleeAttack,
		EIAThrowGrenade,
		EIASprint,
		EIACrawlSpeed,
		EIACycleFireMode,
		EIAInteract,
		EIAToggleParachute,
		EIACycleRadioChannel,
		EIAToggleCamera,
		EIAScoreboardMenu,
		EIAGearUp,
		EIAGearUpOrToggleWeaponLight,
		EIAGearDown,
		EIAGearDownOrExitSupportedShooting,
		EIAClutch,
		EIAHandBrake,
		EIAGiveOrder,
		EIABreathControl,
		EIAMapZoom,
		EIAChangeVehicle,
		EIAChangeEntry,
		EIAChangeEntry1,
		EIAChangeEntry2,
		EIAChangeEntry3,
		EIAChangeEntry4,
		EIAChangeEntry5,
		EIAChangeEntry6,
		EIAChangeEntry7,
		EIAChangeEntry8,
		EIAThreeDimensionalMap,
		EIAShowCommoRose,
		EIAShowLeaderCommoRose,
		EIAQuicktimeInteractDrag,
		EIAQuicktimeFire,
		EIAQuicktimeBlock,
		EIAQuicktimeFastMelee,
		EIAQuicktimeJumpClimb,
		EIAQuicktimeCrouchDuck,
		EIAUndefined,
		EIANoInput,
	};

	enum WeaponAnimType
	{
		WeaponAnimType_NoAddon,
		WeaponAnimType_Bipod,
		WeaponAnimType_Foregrip,
		WeaponAnimType_40mm_GL,
		WeaponAnimType_40mm_GL_Fire,
		WeaponAnimType_Underslung_Shotgun,
		WeaponAnimType_Underslung_Shotgun_Fire,
		WeaponAnimType_Straight_Pull_Bolt,
	};

	enum ZoomLevelActivateEventType
	{
		ZoomLevelActivateEventType_Disable,
		ZoomLevelActivateEventType_Enable,
		ZoomLevelActivateEventType_ToggleOnLightSwitch,
	};

	enum LockType
	{
		LockAlways,
		LockOnRadar,
		LockOnHeat,
		LockOnLaserPainted,
		LockNever,
		LockTypeCount,
	};

	enum WeaponFiringEvent
	{
		WeaponFiringEvent_Push,
		WeaponFiringEvent_Pop,
		WeaponFiringEvent_PrimaryStartedFiringCallback,
		WeaponFiringEvent_PrimaryFireCallback,
		WeaponFiringEvent_PrimaryFireReleaseCallback,
		WeaponFiringEvent_PrimaryFireShotSpawnedCallback,
		WeaponFiringEvent_PrimaryFireAutomaticBeginCallback,
		WeaponFiringEvent_PrimaryFireAutomaticEndCallback,
		WeaponFiringEvent_PrimaryStoppedFiringCallback,
		WeaponFiringEvent_ReloadPrimaryCallback,
		WeaponFiringEvent_ReloadPrimaryEndCallback,
		WeaponFiringEvent_BoltActionCallback,
		WeaponFiringEvent_BoltActionEndCallback,
		WeaponFiringEvent_DetonationSwitchCallback,
		WeaponFiringEvent_HoldAndReleaseReleaseCallback,
		WeaponFiringEvent_UpdateRequired,
	};

	enum FireLogicType
	{
		fltSingleFire,
		fltSingleFireWithBoltAction,
		fltAutomaticFire,
		fltBurstFire,
		fltHoldAndRelease,
		fltDetonatedFiring,
		fltCount,
	};

	enum ReloadLogic
	{
		rlWeaponSwitchCancelsUnfinishedReload,
		rlReloadUnaffectedByWeaponSwitch,
	};

	enum ReloadType
	{
		rtSingleBullet,
		rtMagazine,
		rtMagazineWithPossibleShorterReload,
	};

	namespace ant
	{
		class QuatTransform
		{
		public:
			Vec3 transAndScale;		// 0x00
			Vec3 rotation;			// 0x10
		}; // 0x20

		class Quat
		{
		public:
			enum ZeroType
			{
				Zer                  // constant 0x0
			}; // ZeroType

			enum IdentityType
			{
				Identity                     // constant 0x0
			}; // IdentityType
			Vec4 vec;                     // 0x0
		}; // fb::Quat

		class UpdatePoseResultData
		{
		public:
			QuatTransform* m_localTransforms;				// 0x00
			QuatTransform* m_worldTransforms;				// 0x04
			LPD3DXMATRIX m_renderTransforms;				// 0x08
			QuatTransform* m_interpolatedLocalTransforms;	// 0x0C
			QuatTransform* m_interpolatedWorldTransforms;	// 0x10
			QuatTransform* m_activeWorldTransforms;			// 0x14
			QuatTransform* m_activeLocalTransforms;			// 0x18
			INT m_slot;										// 0x1C
			INT m_readerIndex;								// 0x20
			CHAR m_validTransforms;							// 0x24
			CHAR m_poseUpdateEnabled;						// 0x25
			CHAR m_poseNeeded;								// 0x26
			PAD(0x1);										// 0x27
		}; // 0x28

		class AnimationSkeleton
		{
		public:
			class PoseBlock
			{
				QuatTransform* m_localTransforms;				// 0x00
				QuatTransform* m_worldTransforms;				// 0x04
				LPD3DXMATRIX m_renderTransforms;				// 0x08
				QuatTransform* m_interpolatedWorldTransforms;	// 0x0C
				QuatTransform* m_interpolatedLocalTransforms;	// 0x10
				QuatTransform* m_targetLocalTransforms;			// 0x14
				DWORD m_initData;								// 0x18
				DWORD m_blendData;								// 0x1C
				DWORD m_resultData;								// 0x20
				DWORD m_interpolationData;						// 0x24
				DWORD m_interpolationTicks;						// 0x28
			}; // 0x2C

			fb::SkeletonAsset* m_skeletonAsset;						// 0x00
			DWORD m_boneCount;										// 0x04
			eastl::vector<LPCSTR> m_unknown;						// 0x08
			eastl::vector<LPCSTR> m_boneNames;						// 0x18
			DWORD m_sourceBoneCount;								// 0x28
			DWORD m_dmaAlignedPoseSize2Comp;						// 0x2C
			DWORD m_dmaAlignedPoseSize3Comp;						// 0x30
			DWORD m_dmaAlignedPoseSize4Comp;						// 0x34
			PAD(0xA4);												// 0x38
			eastl::vector<INT> m_activePoseSlots;					// 0xDC
			DWORD m_poseCount;										// 0xEC
			eastl::fixed_vector<PoseBlock*, 16, 2> m_poseBlocks;	// 0xF0
			CHAR m_hasInterpolationTransforms;						// 0x144
			CHAR m_hasRenderTransforms;								// 0x145
			PAD(0x2);												// 0x146
		}; // 0x148
	};

	class TypeInfoData
	{
	public:
		char * name;                     // this+0x0
		USHORT flags;                     // this+0x4
		USHORT totalSize;                     // this+0x6
		void/*ModuleInfo*/ * module;                     // this+0x8

		UCHAR alignment;                     // this+0xC
		UCHAR fieldCount;                     // this+0xD
		UCHAR version;                     // this+0xE
		UCHAR pad;                     // this+0xF

		//    INT marshaledSize;
		void/*FieldInfoData*/ *parent;
		void /*MemberInfoData*/ *members;
	};

	class TypeInfo
	{
	public:
		fb::TypeInfoData * m_infoData;
		TypeInfo * m_pNext;
		USHORT m_uRuntimeId;
	};

	class ITypedObject
	{
	public:
		virtual fb::TypeInfo * getType(); //0x00
	}; // 0x04

	class DataContainer
		: public ITypedObject		// 0x00
	{
	public:
		WORD m_refCnt;				// 0x04
		WORD m_flags;				// 0x06
	}; // 0x08

	class Asset
		: public DataContainer	// 0x00
	{
	public:
		String m_name;			// 0x08
	}; // 0x0C

	class WorldRenderSettings
	{

	public:
		char _0x0000[16];
		float m_dynamicEnvmapDefaultPosition_0; //0x0010
		float m_dynamicEnvmapDefaultPosition_1; //0x0014
		float m_dynamicEnvmapDefaultPosition_2; //0x0018
		float m_dynamicEnvmapDefaultPosition_3; //0x001C
		float m_subSurfaceColor_0; //0x0020
		float m_subSurfaceColor_1; //0x0024
		float m_subSurfaceColor_2; //0x0028
		float m_subSurfaceColor_3; //0x002C
		float m_viewportScale; //0x0030
		float m_shadowMinScreenArea; //0x0034
		float m_shadowViewportScale; //0x0038
		DWORD m_fxaaQuality; //0x003C
		float m_cullScreenAreaScale; //0x0040
		float m_planarReflectionCullFOV; //0x0044
		DWORD m_planarReflectionWidth; //0x0048
		float m_shadowmapSizeZScale; //0x004C
		DWORD m_shadowmapResolution; //0x0050
		DWORD m_shadowmapQuality; //0x0054
		char _0x0058[8];
		float m_shadowmapSliceSchemeWeight; //0x0060
		float m_shadowmapFirstSliceScale; //0x0064
		float m_shadowViewDistance; //0x0068
		char _0x006C[68];
		float m_motionBlurScale; //0x00B0
		float m_motionBlurMax; //0x00B4
		float m_motionBlurNoiseScale; //0x00B8
		DWORD m_motionBlurQuality; //0x00BC
		DWORD m_dynamicEnvmapResolution; //0x00C0
		DWORD m_maxDecalVolumeCount; //0x00C4
		DWORD m_motionBlurMaxSampleCount; //0x00C8
		DWORD m_motionBlurFrameAverageCount; //0x00CC
		float m_motionBlurMaxFrameTime; //0x00D0
		float m_forceMotionBlurDepthCutoff; //0x00D4
		float m_forceMotionBlurCutoffGradientScale; //0x00D8
		DWORD m_multisampleCount; //0x00DC
		float m_multisampleThreshold; //0x00E0
		DWORD m_maxSpotLightShadowCount; //0x00E4
		DWORD m_reflectionEnvmapSize; //0x00E8
		float m_spotLightNearPlane; //0x00EC
		float m_subSurfaceRolloff; //0x00F0
		char _0x00F4[4];
		DWORD m_maxSpotLightCount; //0x00F8
		DWORD m_spotLightShadowmapResolution; //0x00FC
		DWORD m_spotLightShadowmapQuality; //0x0100
		char _0x0104[150];
		bool m_drawFirstPersonModel; //0x019A
		char _0x019B[6];
		bool m_enlightenGlass; //0x01A1
		bool m_drawDebugSkyEnvmap; //0x01A2
		bool m_debugDrawEmitters; //0x01A3
		char _0x01A4[1];
		bool m_depthOfField; //0x01A5
		char _0x01A6[7];
		bool m_debugDrawDepth; //0x01AD
		char _0x01AE[1];
		bool m_lightBrightMap; //0x01AF
		char _0x01B0[1];
		bool m_debugDrawSkyboxAndMask; //0x01B1
		char _0x01B2[4];
		bool m_debugDrawSkyviewBoxes; //0x01B6
		bool m_skyEnable; //0x01B7
		char _0x01B8[6];
		bool m_sunEnabled; //0x01BE
		char _0x01BF[7];
		bool m_fluorescentBloom; //0x01C6
		char _0x01C7[6];
		bool m_drawFoliage; //0x01CD
		char _0x01CE[4];
		bool m_drawLight; //0x01D2
		bool m_unlitEnable; //0x01D3
		char _0x01D4[1];
		bool m_spotLightEnable_0; //0x01D5
		char _0x01D6[3];
		bool m_spotLightEnable_1; //0x01D9
		char _0x01DA[3];
		bool m_lightShadows; //0x01DD
		char _0x01DE[17];
		bool m_clearBuffer; //0x01EF
		char _0x01F0[10];
		bool m_softenSmoke; //0x01FA
		char _0x01FB[1];
		bool m_somethingLighting; //0x01FC
		bool m_drawReflection; //0x01FD
		bool m_fadeDecals; //0x01FE
		char _0x01FF[5];
		bool m_debugDrawMovementOrSomething; //0x0204
		bool m_debugChristmasAcidTrip; //0x0205
		char _0x0206[1];
		bool m_debugMultiView; //0x0207 Has color map, fullbright, shadow, everything!
		char _0x0208[8];
		bool m_lightEnabledAgain; //0x0210
		bool m_drawShadows; //0x0211
		char _0x0212[3];
		bool m_drawNothingButGround; //0x0215
		char _0x0216[10];
		bool m_debugDrawShadowBoxes; //0x0220
		char _0x0221[2];
		bool m_moreShadows; //0x0223
		bool DONTFUCKINGCHANGEME; //0x0224
		char _0x0225[5];
		bool m_smokeShadows; //0x022A
		char _0x022B[7];
		bool m_comeBackToMe2; //0x0232
		bool m_softenLighting; //0x0233
	}; // fb::WorldRenderSettings

	class WorldRenderer
	{
	public:
		class RootView
		{
		public:
			DWORD    index;
			/*WorldDrawViewInfo*   */void* viewInfo;
		};
		void*    m_vtable;
		char _0x0004[32];
		DWORD m_viewWidth; //0x0024
		DWORD m_viewHeight; //0x0028
		char _0x002C[35164];
		WorldRenderSettings* m_worldRenderSettings; //0x8988
	};

	class WorldRenderModule
	{
	public:
		/*unsigned char pad[0x18];				//0000
		//		IWorldRenderer* m_worldRenderer;       // 0x18
		bool m_fmvPlaying;                     // 0x1C
		bool m_worldEnable;                     // 0x1D*/
		char _0x0000[24]; // Inheritance
		WorldRenderer* m_worldRenderer;
		bool m_fmvPlaying;
		bool m_worldEnable;

	}; // fb::WorldRenderModule

	class VegetationManager
	{
	public:
		char _0x0000[4]; // Unknown
		float m_time;
		float m_updateTime;
		float m_updateGhostTime;
		__int32 m_frameCounter;
		__int32 m_currentLocalInfluenceId;
		__int32 m_number_fActiveInstances;
		__int32 m_simulationMemoryInUse;
		char _0x0020[96];
		__int32 m_currentInstanceIndex;
		char _0x0084[652];
		VegetationSystemSettings* m_settings;
	};

	class ClientLevel
		//: public Level//, public IEmitterCollisionHook ,public SimUpdateArgs,public FrameInterpolationUpdateArgs
	{
	public:
		//old
		//char _skipped_0[184]; // PAD(0xB8);0x0000
		// seems i need to pad 28 bytes less now
		//char _skipped_0[156]; //0x0000
		PAD(0x9C);
		DebrisManager*    m_debrisManager;
		PAD(0x4);
		VegetationManager*    m_vegetationManager;
		PAD(0x4);
		EmitterManager*    m_emitterManager;
		void*    m_emitterRenderManager;
		void*    m_decalManager;
		WorldRenderModule* m_worldRenderModule;  // 0xB8
		HavokPhysicsManager*  m_PhysicsManager; //0x00BC
		GameWorld* m_gameWorld; //0x00C0
		void* m_tweaker; //0x00C4 fb::ClientLevelTweaker
		BYTE m_isFinalized; //0x00C8
		BYTE m_hasStartedServer; //0x00C9
		BYTE m_autoRespawn; //0x00CA
		BYTE _ClientLevel_padding; //0x00C
	};//Size=0x00CC(204)

	class GameContext
	{
	public:
		PAD(8);											// 0x00
		PlayerManager* m_pPlayerManager;					// 0x08
		GameTime* m_gameTime;							// 0x0C
		ClientLevel*  m_level;								// 0x10
		MaterialGridManager* m_materialGridManager;		// 0x14
		DWORD m_animationManager;						// 0x18 ant::AnimationManager
		DWORD m_modelAnimationManager;					// 0x1C ModelAnimationManager
		DWORD m_blueprintBundleManager;					// 0x20 BlueprintBundleManager
		DWORD m_dlcManager;								// 0x24 DLCManager
		DWORD m_demoControl;							// 0x28 DemoControl
		INT m_realm;									// 0x2C
	}; // 0x30

	class ClientGameContext
		: public GameContext							// 0x00
	{
	public:
		ClientPlayerManager* m_clientPlayerManager;		// 0x30

	public:
		static ClientGameContext* Singleton()
		{
			return *(ClientGameContext**)(OFFSET_CLIENTGAMECONTEXT);
		}
	}; // 0x34

	class PlayerManager
	{
	public:
		virtual void Function0(); //name: getPlayers returns: const eastl::vector< fb::Player* >
		virtual void Function1(); //name: getSpectators returns: const eastl::vector< fb::Player* >

		char _0x0004[4];
		unsigned int m_maxPlayerCount; //0x0008
		unsigned int m_pPlayerCountBitCount; //0x000C
		unsigned int m_pPlayerIdBitCount; //0x0010

	public:
		class iterator
		{
		public:
			typedef unsigned int type_value;
			typedef unsigned int& reference;
			typedef unsigned int* pointer;

		public:
			iterator(reference ref) : _ref(ref)
			{
			}

			iterator operator ++ ()
			{
				_ref++;

				return *this;
			}

			reference operator * ()
			{
				return _ref;
			}

			type_value index()
			{
				return _ref;
			}

			bool operator == (const iterator& cmp)
			{
				return (_ref == cmp._ref);
			}

			bool operator != (const iterator& cmp)
			{
				return (_ref != cmp._ref);
			}

		private:
			reference _ref;
		};

	public:
		iterator begin() { unsigned int nll = 0; return iterator(nll); }
		iterator end() { return iterator(m_maxPlayerCount); }

	};//Size=0x0014

	class PlayerData
		: public Asset								// 0x00
	{
	public:
		DataContainer* m_pPlayerView;				// 0x0C
		DataContainer* m_inputConceptDefinition;	// 0x10
		DataContainer* m_inputMapping;				// 0x14
	}; // 0x18

	class GameTime
	{
	public:
		DWORD m_ticks;						// 0x00
		DWORD m_tickFrequency;				// 0x04
		DWORD m_tickIndexInFrame;			// 0x08
		DWORD m_lastTickIndexInFrame;		// 0x0C
		DWORD m_tickCountInFrame;			// 0x10
		FLOAT m_deltaTime;					// 0x14
		FLOAT m_passedDeltaTimeInFrame;		// 0x18
		DOUBLE m_time;						// 0x1C
		INT m_useVariableDeltaTime;			// 0x24
	}; // 0x28

	class SubLevel
	{
	public:
		LPVOID vftable;											// 0x00
		eastl::vector<EntityCollectionSegment*> m_segments;		// 0x04
		eastl::vector<UINT> m_deletedEntities;					// 0x14
		Blueprint* m_subLevelData;								// 0x24
		SubLevel* m_parent;										// 0x28
		SubLevel* m_child;										// 0x2C
		SubLevel* m_sibling;									// 0x30
		MemoryArena* m_arena;									// 0x34
		ResourceCompartment m_compartment;						// 0x38
		Realm m_realm;											// 0x3C
		INT m_refCount;											// 0x40
		BYTE m_isDestroyed;										// 0x44
		PAD(3);									// 0x45
	}; // 0x48

	class EntityCollectionSegment
		: public eastl::vector<Entity*>                // 0x00
	{
	public:
		//eastl::vector<fb::Entity *> m_Collection;   // 0x00
		SubLevel* m_subLevel;						// 0x10
		EntityCollectionSegment* m_next;			// 0x14
		EntityCollectionSegment* m_prev;			// 0x18
		DWORD m_iterableSize;						// 0x1C
		DWORD m_collectionIndex;					// 0x20
	}; // 0x24

	class PropertyModificationListener
		: public ITypedObject			// 0x00
	{
	}; // 0x04

	class EntityBusPeer
		: public PropertyModificationListener		// 0x00
	{
	}; // 0x04

	class Entity
		: public EntityBusPeer			// 0x00
	{
	public:
		DWORD m_weakTokenHolder;		// 0x04
		DWORD m_flags;					// 0x08
	}; // 0x0C

	class DataBusData
		: public Asset										// 0x00
	{
	public:
		Array<PropertyConnection> m_propertyConnections;	// 0x0C
		Array<LinkConnection> m_linkConnections;			// 0x10
	}; // 0x14

	class PropertyConnection
	{
	public:
		DataContainer* m_source;		// 0x00
		DataContainer* m_target;		// 0x04
		INT m_sourceFieldId;			// 0x08
		INT m_targetFieldId;			// 0x0C
	}; // 0x10

	class LinkConnection
		: public PropertyConnection		// 0x00
	{
	}; // 0x10

	class EntityBusData
		: public DataBusData						// 0x00
	{
	public:
		Array<EventConnection> m_eventConnections;	// 0x14
		DataContainer* m_descriptor;				// 0x18
		BYTE m_needNetworkid;						// 0x1C
		BYTE m_interfaceHasConnections;				// 0x1D
		BYTE m_hasNetworkedEvents;					// 0x1E
		PAD(0x1);									// 0x1F
	}; // 0x20

	class EventConnection
	{
	public:
		DataContainer* m_source;		// 0x00
		DataContainer* m_target;		// 0x04
		INT m_sourceEvent;				// 0x08
		INT m_targetEvent;				// 0x0C
		INT m_targetType;				// 0x10
	}; // 0x14

	class Blueprint
		: public EntityBusData			// 0x00
	{
	}; // 0x20

	class LevelSetupOption
	{
		String m_criterion;		// 0x00
		String m_value;			// 0x04
	}; // 0x08

	class LevelSetup
	{
	public:
		String m_name;									// 0x00
		Array<LevelSetupOption> m_inclusionOptions;		// 0x04
		DWORD m_difficultyIndex;						// 0x08
		PAD(0xC);										// 0x0C
	}; // 0x18

	class Level
		: public SubLevel								// 0x00
	{
	public:
		GUID m_checksum;								// 0x48
		MaterialGridManager* m_materialGridManager;		// 0x58
		EntityBus* m_entityBus;							// 0x5C
		LevelData* m_data;								// 0x60
		TeamInfo* m_teamInfo;							// 0x64
		LevelSetup m_levelSetup;						// 0x68
	}; // 0x80

	class Level2
	{
	public:
		char Unknowns01[0xBC]; // 0x0
		HavokPhysicsManager* m_physicsManager; // 0xBC
		GameWorld* m_gameWorld; // 0xC0
	};
	class DebrisManager
	{
	public:
		char _0x0000[4];
		int m_refCount; //0x0004
		char _0x0008[100];
		float m_time; //0x006C
		int m_frameCount; //0x0070
		int m_totalPartsSpawnedCount; //0x0074
		char _0x0078[8];
		DWORD m_debrisArena; //0x0080
		//DebrisSystemSettings* m_settings; //0x0084
		char _0x0088[112];

	};//Size=0x00F4


	class ClientVegetationManager
	{
	public:
		char _0x0000[4];
		float m_time; //0x0004
		float m_updateTime; //0x0008
		float m_updateGhostTime; //0x000C
		int m_frameCounter; //0x0010
		int m_currentLocalInfluenceID; //0x0014
		int m_number_fActiveInstances; //0x0018
		int m_symulationMemoryInUse; //0x001C
		char _0x0020[96];
		int m_currentInstanceIndex; //0x0080
		char _0x0084[652];

	};//Size=0x033C

	class VegetationSystemSettings
	{
	public:
		char _0x0000[8];
		float m_windVariation; //0x0008
		float m_windVariationRate; //0x000C
		float m_windStrength; //0x0010
		float m_jointTentionLimit; //0x0014
		int m_forceShadowLod; //0x0018
		int m_maxPreSimsPerJob; //0x001C
		int m_simulationMemKbServer; //0x0020
		float m_maxActiveDistance; //0x0024
		int m_simulationMemKbClient; //0x0028
		float m_timeScale; //0x002C
		int m_jobCount; //0x0030
		int m_jointTentionLimitIndex; //0x0034
		bool m_simulateServerSide; //0x0038
		bool m_localInfluenceEnable; //0x0039
		bool m_useShadowLodOffset; //0x003A
		bool m_destructionEnabled; //0x003B
		bool m_enableJobs; //0x003C
		bool m_enable; //0x003D
		bool m_drawNodes; //0x003E
		bool m_batchDrawEnable; //0x003F
		bool m_drawEnable; //0x0040
		bool m_dissolveEnable; //0x0041
		bool m_shadowMeshEnable; //0x0042

	};//Size=0x0043

	class EmitterManager
	{
	public:
		char _0x0000[4];
		int m_refCount; //0x0004
		char _0x0008[124];
		int m_jobCountPerStage; //0x0084
		char _0x0088[440];
		bool m_enable; //0x0240
		bool m_fixedDeltaEnable; //0x0241
		char _0x0242[2];
		float m_globalResetStartTimeInterval; //0x0244
		float m_timeScale; //0x0248
		float m_globalResetStartTime; //0x024C

	};//Size=0x0250

	class DebrisSystemSettings
	{
	public:
		char _0x0000[8];
		int m_meshDrawCountLimit; //0x0008
		float m_timeScale; //0x000C
		float m_meshCullingDistance; //0x0010
		int m_drawStats; //0x0014
		char _0x0018[4];
		float m_meshStreamingPriorityMultiplier; //0x001C
		bool m_meshRenderingEnable; //0x0020
		bool m_enable; //0x0021
		bool m_meshShadowEnable; //0x0022
		bool m_meshDrawBoundingBoxes; //0x0023
		bool m_meshViewCullingEnable; //0x0024
		bool m_meshHavocRenderingEnable; //0x0025
		bool m_enableJobs; //0x0026
		bool m_meshDrawTransforms; //0x0027

	};//Size=0x0028

	class Level3
	{
	public:
		char _0x0000[60];
		int m_realm; //0x003C
		int m_refCount; //0x0040
		bool m_isDestroyed; //0x0044
		char _0x0045[87];
		DebrisManager* m_debrisManager; //0x009C
		char _0x00A0[4];
		ClientVegetationManager* m_vegetationManager; //0x00A4
		char _0x00A8[4];
		EmitterManager* m_emitterManager; //0x00AC
		char _0x00B0[84];

	};//Size=0x0104




	class MaterialGridManager
	{
	public:
		EntityBusData* m_busData;						// 0x00
		EntityBus* m_entityBus;							// 0x04
		MaterialGridData* m_data;						// 0x08
		MaterialContainerPair* m_defaultMaterialPair;	// 0x0C
		DWORD m_defaultMaterialIndex;					// 0x10
	}; // 0x14

	class EntityBus
	{
	public:
		LPVOID vftable;							// 0x00
		SubLevel* m_subLevel;					// 0x04
		EntityBus* m_parentBus;					// 0x08
		INT m_dataId;							// 0x0C
		INT m_refCount;							// 0x10
		INT m_networkId;						// 0x14
		SHORT m_realm;							// 0x18
		CHAR m_parentPropertiesCanChange;		// 0x1A
		PAD(0x1);								// 0x1B
	}; // 0x1C

	class MaterialGridData
		: public Asset												// 0x00
	{
	public:
		DataContainer* m_defaultMaterial;							// 0x0C
		RefArray<MaterialContainerPair> m_materialPairs;			// 0x10
		Array<UINT> m_materialIndexMap;								// 0x14
		DWORD m_defaultMaterialIndex;								// 0x18
		Array<MaterialRelationPropertyPair> m_materialProperties;	// 0x1C
		Array<MaterialInteractionGridRow> m_interactionGrid;		// 0x20
	}; // 0x24

	class MaterialContainerPair
		: public DataContainer			// 0x00
	{
	public:
		DWORD m_flagsAndIndex;			// 0x08
		BYTE m_physicsPropertyIndex;	// 0x0C
		BYTE m_physicsMaterialIndex;	// 0x0D
		PAD(0x2);						// 0x0E
	}; // 0x10

	class MaterialRelationPropertyPair
	{
	public:
		RefArray<DataContainer> m_physicsMaterialProperties;		// 0x00
		RefArray<DataContainer> m_physicsPropertyProperties;		// 0x04
	}; // 0x08

	class MaterialInteractionGridRow
	{
	public:
		RefArray<MaterialRelationPropertyPair> m_items;				// 0x00
	}; // 0x04

	class PrefabBlueprint
		: public Blueprint						// 0x00
	{
		RefArray<GameObjectData> m_objects;		// 0x20
	}; // 0x24

	class GameDataContainer
		: public DataContainer			// 0x00
	{
	}; // 0x08

	class GameObjectData
		: public GameDataContainer				// 0x00
	{
	public:
		WORD m_indexInBlueprint;				// 0x08
		BYTE m_isEventConnectionTarget;			// 0x0A
		BYTE m_isPropertyConnectionTarget;		// 0x0B
	}; // 0x0C

	class SpatialPrefabBlueprint
		: public PrefabBlueprint				// 0x00
	{
	}; // 0x24

	class SubWorldData
		: public SpatialPrefabBlueprint			// 0x00
	{
	public:
		DataContainer* m_registryContainer;		// 0x24
		BYTE m_isWin32SubLevel;					// 0x28
		BYTE m_isXenonSubLevel;					// 0x29
		BYTE m_isPs3SubLevel;					// 0x2A
		PAD(0x1);								// 0x2B
	}; // 0x2C

	class WorldData
		: public SubWorldData					// 0x00
	{
	public:
		DataContainer* m_runtimeMaterialGrid;	// 0x2C
	}; // 0x30

	class PathfindingBlob
	{
	public:
		GUID m_blobId;				// 0x00
		DWORD m_blobSize;			// 0x10
		Array<UINT> m_chunkSizes;	// 0x14
	}; // 0x18

	class LevelDescription
	{
	public:
		String m_name;							// 0x00
		String m_description;					// 0x04
		RefArray<DataContainer> m_components;	// 0x08
		BYTE m_isCoop;							// 0x0C
		BYTE m_isMenu;							// 0x0D
		BYTE m_isMultiplayer;					// 0x0E
		PAD(0x1);								// 0x0F
	}; // 0x10

	class LevelData
		: public WorldData								// 0x00
	{
	public:
		DataContainer* m_levelReference;				// 0x30
		PathfindingBlob m_blob;							// 0x34
		DataContainer* m_aiSystem;						// 0x4C
		FLOAT m_worldSizeXZ;							// 0x50
		LevelDescription m_description;					// 0x54
		String m_gameConfigurationName;					// 0x64
		DataContainer* m_emitterSystemAsset;			// 0x68
		RefArray<DataContainer> m_exclusionVolumes;		// 0x6C
		FLOAT m_defaultFov;								// 0x70
		FLOAT m_infantryFovMultiplier;					// 0x74
		DWORD m_maxEntityBusNetworkCount;				// 0x78
		DataContainer* m_soundStates;					// 0x7C
		DataContainer* m_voiceOverSystem;				// 0x80
		RefArray<Asset> m_voiceOverLogic;				// 0x84
		FLOAT m_maxVehicleHeight;						// 0x88
		String m_aerialHeightmapData;					// 0x8C
		DataContainer* m_enlightenShaderDatabase;		// 0x90
		RefArray<Asset> m_antProjectAssets;				// 0x94
		DataContainer* m_faceAnimationWaveMappings;		// 0x98
		DataContainer* m_animatedSkeletonDatabase;		// 0x9C
		RefArray<Asset> m_cameraModes;					// 0xA0
		RefArray<DataContainer> m_cameraTransitions;	// 0xA4
		DataContainer* m_hackForceBuild;				// 0xA8
		BYTE m_hugeBroadPhase;							// 0xAC
		BYTE m_freeStreamingEnable;						// 0xAD
		PAD(0x2);										// 0xAE
	}; // 0xB0

	class TeamInfo
	{
	public:
		TeamEntityData* m_teams[17];			// 0x00
		UINT m_teamCount;						// 0x44
	};

	class EntityData
		: public GameObjectData					// 0x00
	{
	}; // 0xC

	class SpatialEntityData
		: public EntityData				// 0x00
	{
	public:
		PAD(0x4);						// 0x0C
		D3DXMATRIX m_transform;	// 0x10
	}; // 0x50

	class GameEntityData
		: public SpatialEntityData				// 0x00
	{
	public:
		RefArray<GameObjectData> m_components;	// 0x50
		BYTE m_enabled;							// 0x54
		BYTE m_runtimeComponentCount;			// 0x55
		PAD(0xA);								// 0x56
	}; // 0x60

	class TeamEntityData
		: public GameEntityData			// 0x00
	{
	public:
		DataContainer* m_team;			// 0x60
		INT m_id;						// 0x64
	}; // 0x68

	class ClientPlayerManager
		: public PlayerManager,								// 0x00
		public network::ClientGhost,						// 0x14
		public network::Interpolator<LPVOID>,				// 0x54
		public network::IClientNetworkable					// 0x94
	{
	public:
		eastl::vector<ClientPlayer*> m_pPlayers;				// 0x9C
		eastl::vector<ClientPlayer*> m_spectators;			// 0xAC
		ClientPlayer* m_localPlayer;						// 0xBC
		ClientPlayer** m_idToPlayerMap;						// 0xC0
		PAD(0x4);											// 0xC4
		ClientPlayerManagerPlayer* m_networkablePlayers;	// 0xC8
	};

	class ClientPlayerManagerPlayer
		: public network::IClientNetworkableGroupMember		// 0x00
	{
	public:
		ClientPlayerManager* m_manager;						// 0x08
		DWORD m_id;											// 0x0C
	}; // 0x10

	class OnlineId
	{
	public:
		ULONGLONG m_nativeData;		// 0x00
		CHAR m_szId[0x11];			// 0x08
		PAD(0x7);					// 0x19
	}; // 0x20

	class Player
	{
	public:
		LPVOID vftable;							// 0x00
		DWORD m_weakTokenHolder;				// 0x04
		PlayerData* m_data;						// 0x08
		MemoryArena* m_arena;					// 0x0C
		eastl::basic_string<CHAR> m_name;//eastl::basic_string<CHAR> m_name;		// 0x10
		OnlineId m_onlineId;					// 0x20
		OnlineId m_groupId;						// 0x40
		OnlineId m_clubId;						// 0x60
		INT m_teamPreference;					// 0x80
		PAD(0x298);								// 0x84
		INT m_teamId;							// 0x31C
	}; // 0x320

	class WeaponFiringShooter
	{
	public:
		LPVOID vftable;				// 0x00
		DWORD m_weakTokenHolder;	// 0x04
	}; // 0x08

	class BitArray
	{
	public:
		LPVOID vftable;					// 0x00
		LPINT m_bits;					// 0x04
		DWORD m_externalBufferSize;		// 0x08
		DWORD m_bitCount;				// 0x0C
		DWORD m_wordCount;				// 0x10
	}; // 0x14

	enum SpotType;
	enum GamePlayBones;
	enum TeamId;
	enum SquadId;
	enum GamePlatform;
	enum CameraIds;
	enum FreeCameraMode;
	enum UISystemType;
	enum ClientState;
	enum ClientGameType;
	enum InputConceptIdentifiers;

	class ClientPlayer2
	{
	public:
		virtual void Function0(); //
		virtual fb::SoldierEntity* getSoldier(); // fb::SoldierEntity
		virtual void* getEntry(); //
		virtual bool isInVehicle(); //
		virtual unsigned int getId(); //
		virtual void Function5(); //
		virtual void Function6(); //
		virtual void Function7(); //
		virtual void Function8(); //
		virtual void Function9(); //

		char _0x0004[36];
		char m_name[16]; //0x0028
		char _0x0038[72];
		__int32 m_teamPreference; //0x0080
		__int32 m_lastVehicleCameraIndex; //0x0084
		__int32 m_analogInputEnableMask; //0x0088
		char _0x008C[656];
		int m_teamId;// fb::TeamId m_teamId; //0x031C
		bool m_isAIPlayer; //0x0320
		bool m_isSpectator; //0x0321
		char _0x0322[158];
		fb::WeakPtr< fb::ClientSoldierEntity > m_soldier; //0x03C0
		fb::WeakPtr< fb::ClientSoldierEntity > m_corpse; //0x03C4
		fb::WeakPtr< /*fb::ClientCharacterEntity*/fb::CharacterEntity > m_character; //0x03C8
		char _0x03CC[4];
		ClientControllableEntity* m_attachedControllable; //0x03D0
		__int32 m_attachedEntryId; //0x03D4
		ClientControllableEntity* m_controlledControllable; //0x03D8
		char _0x03DC[144];
		int m_squadId;//fb::SquadId m_squadId; //0x046C
		bool m_isSquadLeader; //0x0470
		bool m_isSquadPrivate; //0x0471
		bool m_isAllowedToSpawnOn; //0x0472
		bool m_isAdmin; //0x0473
		char _0x0474[204];

	public:
		fb::ClientSoldierEntity* getClientSoldierEntity()
		{
			DWORD dwSoldier = (DWORD)this->getSoldier();

			if (dwSoldier)
				return (fb::ClientSoldierEntity*)(dwSoldier - 0xF0);

			return NULL;
		}

		fb::ClientControllableEntity* getControllable()
		{
			if (m_attachedControllable)
				return m_attachedControllable;
			else if (m_controlledControllable)
				return m_controlledControllable;

			return NULL;
		}

	};//Size=0x0540

	class ClientPlayer
		: public Player,										// 0x00
		public network::ClientGhost,							// 0x320
		public network::IClientNetworkable,						// 0x360
		public network::Interpolator<LPVOID>					// 0x368
	{
	public:
		class ClientPlayerShooter
			: public WeaponFiringShooter		// 0x00
		{
		public:
			ClientPlayer* m_pPlayer;				// 0x08
		}; // 0x0C

		BitArray m_unlocksBitArray;								// 0x3A8
		INT m_refCount;											// 0x3BC
		WeakPtr<ClientSoldierEntity> m_soldier;					// 0x3C0
		WeakPtr<ClientSoldierEntity> m_corpse;					// 0x3C4
		WeakPtr<ClientCharacterEntity> m_character;				// 0x3C8
		DWORD m_weakTokenHolder;								// 0x3CC
		ClientControllableEntity* m_attachedControllable;		// 0x3D0
		DWORD m_attachedEntryId;								// 0x3D4
		ClientControllableEntity* m_controlledControllable;		// 0x3D8
		DWORD m_controlledEntryId;								// 0x3DC
		EntryInput* m_input;									// 0x3E0
		EntryInputState* m_inputState;							// 0x3E4
		EntryInputState* m_externalInputState;					// 0x3E8
		DWORD m_id;												// 0x3EC
		DWORD m_connectionId;									// 0x3F0
		INT m_lastTeamHit;										// 0x3F4
		INT m_inputNetworkId;									// 0x3F8
		ClientEntryComponent* m_oldEntry;						// 0x3FC
		ClientPlayerShooter* m_shooter;							// 0x400
		ClientPlayerManager* m_pPlayerManager;					// 0x404
		ClientPlayerView* m_ownPlayerView;						// 0x408
		ClientPlayerView* m_pPlayerView;							// 0x40C
		PAD(0x4);												// 0x410
		EntryInputActionMap* m_inputActionMap;					// 0x414
		EntryInputTranslator* m_inputTranslator;				// 0x418
		WORD m_finalScore;										// 0x41C
		WORD m_finalScoreSet;									// 0x41E
	}; // 0x420

	class EntryInputState
	{
	public:
		class CharacterMeleeIdentifier
		{
		public:
			WeakPtr<Entity> entity;				// 0x00
			network::ClientGhost* clientGhost;	// 0x04
		}; // 0x08

		class CharacterCollisionPos
		{
		public:
			WeakPtr<Entity> entity;					// 0x00
			network::ClientGhost* clientGhost;		// 0x04
			PAD(0x8);								// 0x08
			Vec3 pos;								// 0x10
		}; // 0x20

		LPVOID vftable;									// 0x00
		PAD(0xC);										// 0x04
		FLOAT m_analogInput[10];						// 0x10
		FLOAT m_downTimes[100];							// 0x38
		PAD(0x24);										// 0x1C8
		FLOAT m_deltaTime;								// 0x1EC
		FLOAT m_timeBehind;								// 0x1F0
		FLOAT m_authorativeAimingYaw;					// 0x1F4
		FLOAT m_authorativeAimingPitch;					// 0x1F8
		FLOAT m_authorativeMovementPenalty;				// 0x1FC
		Vec3 m_authoritativeCameraPosition;				// 0x200
		Vec3 m_authoritativeMovementPosition;			// 0x210
		Vec3 m_authoritativeMovementVelocity;			// 0x220
		CharacterMeleeIdentifier m_meleeIdentifier;		// 0x230
		PAD(0x4);										// 0x238
		DWORD m_characterCollisionCount;				// 0x23C
		CharacterCollisionPos m_collisionPos[4];		// 0x240
		DWORD m_zoomLevel;								// 0x2C0
		DWORD m_ticks;									// 0x2C4
		DWORD m_controllableId;							// 0x2C8
		DWORD m_entryId;								// 0x2CC
		DWORD m_networkSequenceNumber;					// 0x2D0
	}; // 0x2D4

	class EntryInput
		: public EntryInputState			// 0x00
	{
	}; // 0x2D4

	class Component
		: public EntityBusPeer						// 0x00
	{
	public:
		ComponentData* m_data;				// 0x04
		ComponentCollection* m_collection;	// 0x08
		PAD(0x4);							// 0x0C
	}; // 0x10

	class ComponentData
		: public GameObjectData						// 0x00
	{
		//LinearTransform m_transform;				// 0x10
		fb::Vec3 m_transform;				// 0x10
		RefArray<GameObjectData> m_components;		// 0x50
		INT m_excluded;								// 0x54
	}; // 0x58

	class ComponentInfo
	{
	public:
		fb::Component* component; //0x0000
		fb::Component* entry; //0x0004
		DWORD flags; //0x0008
		char _0x000C[4];

	};//Size=0x0010

	class ComponentCollection
	{
	public:
		GameEntity* owner; //0x0000
		BYTE playerCount; //0x0004
		BYTE totalCount; //0x0005
		BYTE offsetCount; //0x0006
		char _0x0007[1];

	public:
		ComponentInfo* getInfo(int idx)
		{
			ComponentInfo* info = (ComponentInfo*)this;

			return &(info[idx + 1]);
		}

	};//Size=0x0008

	class ClientComponent
		: public Component		// 0x00
	{
	}; // 0x10


	class ClientWeaponComponent : public ClientComponent
	{
	public:
		PAD(0x50);
		class fb::ClientWeaponComponent/*::Interpolator*/ * m_interpolationObject;                     // this+0x60
		class fb::ClientWeapon * m_weapon;                     // this+0x64
		class fb::ClientWeaponComponent/*Simulation*/ * m_controller;                     // this+0x68
		class fb::ClientWeaponComponent/*Replication*/ * m_replicatedWeapon;                     // this+0x6C
		class fb::ClientWeaponComponent/*Prediction*/ * m_predictedWeapon;                     // this+0x70
		class fb::ClientLockingController * m_lockingController;                     // this+0x74
		class fb::ClientLockingController * m_secondaryLockingController;                     // this+0x78
		class fb::ClientPlayer * m_pPlayer;                     // this+0x7C

		class WeaponInfo
		{
		public:
			//class EntryComponent::WeaponInfo // Inherited class at offset 0x0
			//{
			//	//LPVOID vftable;		// 0x00
			//	virtual void worldTransform(struct fb::LinearTransform &);	// V: 0x0
			//	virtual void getState(class fb::WeaponFiring &);	// V: 0x4
			//	virtual void addWeaponFiringCallbacks(class fb::WeaponFiringCallbacks *);	// V: 0x8
			//	virtual void removeWeaponFiringCallbacks(class fb::WeaponFiringCallbacks *);	// V: 0xC
			//	virtual class WeaponFiring * weaponFiring();	// V: 0x10
			//	virtual class Weapon * weapon();	// V: 0x14
			//
			//}; // fb::EntryComponent::WeaponInfo
			PAD(0x4);

			class ClientWeaponComponent *m_weapon;                     // this+0x4
			class ClientWeaponComponent/*Replication*/ & m_replicated;                     // this+0x8
			class ClientWeaponComponent/*Prediction*/ & m_predicted;                     // this+0xC
			class ClientStanceFilterComponent * m_stanceFilterComponent;                     // this+0x10

		}; // WeaponInfo

		class ClientWeaponComponent::WeaponInfo * m_weaponInfo;                     // this+0x80
		//class ClientWeaponFiringCallbacks
		//{

		//	class WeaponFiringCallbacks // Inherited class at offset 0x0
		//	{

		//		enum CallOrder
		//		{

		//			int CallOrder_PreWeapon;                     // constant 0x0
		//			int CallOrder_Weapon;                     // constant 0x1
		//			int CallOrder_PostWeapon;                     // constant 0x2

		//		}; // CallOrder

		//
		//	}; // fb::WeaponFiringCallbacks

		//			class fb::ClientWeaponComponent & m_weapon;                     // this+0x4

		//}; // ClientWeaponFiringCallbacks

		class ClientWeaponComponent/*::ClientWeaponFiringCallbacks*/ * m_weaponFiringCallback;                     // this+0x84
		class /*MeshModel*/MessageListener * m_weaponMeshModel;                     // this+0x88
		class ClientEntryComponent * m_firingEntry;                     // this+0x8C
		unsigned int m_meshTransformDirty;                     // this+0x90, len(0x4)
		unsigned int m_isFiring;                     // this+0x94, len(0x1)
		unsigned int m_hadInitialAnimUpdate;                     // this+0x95, len(0x1)
		unsigned int m_animHadVisualUpdate;                     // this+0x96, len(0x1)
		unsigned int m_replicatedBarrelIndex;                     // this+0x94
		unsigned int m_predictedBarrelIndex;                     // this+0x98

	}; // fb::ClientWeaponComponent


	class EntryComponent
	{
	public:
		class WeaponInfo
		{
		public:
			//LPVOID vftable;		// 0x00
			virtual void worldTransform(struct fb::LinearTransform &);	// V: 0x0
			virtual void getState(class fb::WeaponFiring &);	// V: 0x4
			virtual void addWeaponFiringCallbacks(class fb::WeaponFiringCallbacks *);	// V: 0x8
			virtual void removeWeaponFiringCallbacks(class fb::WeaponFiringCallbacks *);	// V: 0xC
			virtual class fb::WeaponFiring * weaponFiring();	// V: 0x10
			virtual class fb::Weapon * weapon();	// V: 0x14
		}; // 0x04

		class Subscriber
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04

		class EntrySorter
		{


		}; // EntrySorter

		class State
		{
		public:
			int m_activeStance;                     // this+0x0
			int m_previousStance;                     // this+0x4
			bool m_stanceResetOnExit;                     // this+0x8

		}; // State

		class FiringCallbacks
		{
		public:
			LPVOID vftable;				// 0x00
			EntryComponent* m_entry;	// 0x04
			ClientWeaponComponent::WeaponInfo* m_info;			// 0x08
		}; // 0x0C

		LPVOID vftable;							// 0x00
		//virtual unsigned int getActiveStance();		// V: 0x0
		eastl::vector<FiringCallbacks*> m_weapons;	// 0x04
		eastl::vector<Subscriber*> m_subscribers;	// 0x14
		PAD(0x4);									// 0x24
	}; // 0x28





	class MessageListener
	{
	public:
		PAD(0x8);	// 0x00
	}; // 0x08

	class StanceFilterComponentData
	{
	public:
		PAD(0x60);
		//class Array<int> m_validStances;                     // this+0x60
		int *m_validStances;                     // this+0x60
		float m_stanceChangeTime;                     // this+0x64
		PAD(0x4);
		bool m_filterSpecificActions;                     // this+0x6C
		bool m_undoParentStanceFilter;                     // this+0x6D

	}; // fb::StanceFilterComponentData

	class StanceFilterComponent // Inherited class at offset 0x10
	{

		virtual const class StanceFilterComponentData & stanceFilterData();	// V: 0x4
		class StanceFilterComponent * m_previousStanceFilter;                     // this+0x4
		class EntryComponent * m_controllingEntry;                     // this+0x8
		float m_stanceSwitchTimer;                     // this+0xC
		class eastl::map<enum fb::EntryInputActionEnum, float> m_cachedEntryInputActionMap;                     // this+0x10

	}; // fb::StanceFilterComponent


	class ClientStanceFilterComponent/* :
									 public StanceFilterComponent, public ClientComponent*/
	{
	public:
		PAD(0x4);																	//0x00
		StanceFilterComponentData *StanceFilterData;
		PAD(0x30);
		const class ClientEntryComponent * m_controllingEntry;                     // this+0x38

	}; // fb::ClientStanceFilterComponent

	class ClientEntryComponent
		: public ClientComponent,								// 0x00
		public EntryComponent,									// 0x10
		public network::IClientNetworkable,						// 0x38
		public network::Interpolator<LPVOID>,					// 0x40
		public MessageListener									// 0x80
	{
	public:
		class Camera
		{
		public:
			DWORD camera;				// 0x00
			DWORD callback;				// 0x04
			CHAR isFirstPerson;			// 0x08
			CHAR receivesImpulses;		// 0x09
			PAD(0x2);					// 0x0A
		}; // 0x0C

		DWORD m_turrentComponent;								// 0x88
		PAD(0x4);												// 0x8C
		eastl::vector<EntryInputActionMap*> m_inputActionMaps;  // 0x90
		EntryInputTranslator* m_inputTranslator;				// 0xA0
		eastl::vector<Camera> m_cameras;						// 0xA4
		class CameraScene * m_cameraScene;                  // 0xB4
		class eastl::vector<fb::ClientCameraContext *> m_cameraComponents;                  // this+0xB8
		PAD(0x58);
		class eastl::vector<fb::ClientStanceFilterComponent *> m_stanceFilterComponents;    // this+0xC8
		class ClientPlayer * m_pPlayer;                     // this+0xD8
		unsigned int m_previousCamera;                     // this+0xDC
		bool m_defaultCameraView;                     // this+0xE0
		struct EntryComponent::State m_predictionState;                     // this+0xE4
		struct EntryComponent::State m_correctionState;                     // this+0xF0
		struct EntryComponent::State * m_currentState;                     // this+0xFC
		struct EntryComponent::State m_replicatedState;                     // this+0x100
		//class fb::ClientEntryComponentSound * m_entryComponentSound;                     // this+0x10C
		PAD(0x4);
	}; // 0xB4

	class InputActionMappingsData
		: public DataContainer				// 0x00
	{
	public:
		RefArray<DataContainer> m_mappings;	// 0x08
	}; // 0x0C

	class InputActionMap
	{
	public:
		eastl::vector<InputActions> m_actions;			// 0x00
	}; // 0x10

	class InputActions
	{
	public:
		eastl::vector<InputAction> m_inputActions;		// 0x00
	}; // 0x10

	class InputAction
	{
	public:
		DataContainer* m_data;					// 0x00 InputActionData
		InputConceptIdentifiers m_concept;		// 0x04
	}; // 0x08

	class EntryInputActionMap
		: public InputActionMap				// 0x00
	{
	public:
		LPVOID vftable;						// 0x10
		InputActionMapSlot m_slot;			// 0x14
	}; // 0x18

	class ClientPlayerView
		: public network::ClientGhost,									// 0x00
		public network::Interpolator<LPVOID>,							// 0x40
		public network::IClientNetworkable								// 0x80
	{
	public:
		eastl::fixed_vector<ClientSubView*, 4, 2> m_subViews;			// 0x88
		ClientPlayer* m_owner;											// 0xAC
		eastl::fixed_vector<WeakPtr<ClientPlayer>, 8, 2> m_spectators;	// 0xB0
		FLOAT m_giveDamageTime;											// 0xE4
		FLOAT m_damageGivenToDamageEntity;								// 0xE8
		WeakPtr<ClientVehicleEntity> m_vehicleInteractionEntity;		// 0xEC
		DWORD m_lastWeaponPickupTick;									// 0xF0
		DWORD m_lastAmmoPickupTick;										// 0xF4
		PAD(0x8);														// 0xF8
		Vec3 m_soldierHitPosition;										// 0x100 Ignore Z
		Vec3 m_soldierHitDirection;										// 0x110 Ignore Z
		Vec3 m_soldierHitGiverOrigin;									// 0x120 Ignore Z
		Vec3 m_vehicleHitDirection;										// 0x130 Ignore Z
		FLOAT m_soldierHitDamage;										// 0x140
		CHAR m_isBulletDamage;											// 0x144
		CHAR m_hasSuppressedEnemy;										// 0x145
		PAD(0x2);														// 0x146
		FLOAT m_lockAmount;												// 0x148
		CHAR m_vehicleHitDirectionUpdated;								// 0x14C
		CHAR m_squadSpawnAllowed;										// 0x14D
		PAD(0x2);														// 0x14E
		FLOAT m_spawnIsAllowedTimer;									// 0x150
		FLOAT m_warmUpTimer;											// 0x154
		FLOAT m_timeToRespawn;											// 0x158
		FLOAT m_lastRespawnTime;										// 0x15C
		ClientCameraContext* m_cameraContext;							// 0x160
	}; // 0x164

	class SubView
		: public ITypedObject			// 0x00
	{
	public:
		DataContainer* m_data;			// 0x04 SubViewData
	}; // 0x08

	class ClientSubView
		: public SubView,						// 0x00
		public network::IClientNetworkable		// 0x08
	{
	public:
		ClientPlayerView* m_pPlayerView;			// 0x10
	}; // 0x14

	class CameraContext
	{
	public:
		PAD(0x8);			// 0x00
		LinearTransform m_transform;			// 0x08
		LinearTransform m_targetTransform;		// 0x48
		CameraIds m_cameraId;					// 0x88
		DWORD m_cameraActivePosition;			// 0x8C
	}; // 0x90

	class ClientCameraContext
		: public network::IClientNetworkable,						// 0x00
		public network::Interpolator<LPVOID>,						// 0x08
		public CameraContext										// 0x48
	{
	public:
		PAD(0x8);													// 0xD8
		DWORD m_weakTokenHolder;									// 0xE0
		WeakPtr<ClientControllableEntity> m_targetObject;			// 0xE4
		WeakPtr<ClientControllableEntity> m_targetControllable;		// 0xE8
		DWORD m_targetEntryId;										// 0xEC
		ClientGameView* m_gameView;									// 0xD0
	}; // 0xD4

	class TargetCameraCallback
		: public ITypedObject		// 0x00
	{
	}; // 0x04

	class GameView
		: public TargetCameraCallback		// 0x00
	{
	public:
		class EntryUnSpawnCallback
			: public EntryComponent::Subscriber		// 0x00
		{
		public:
			GameView* m_gameView;					// 0x04
			EntryComponent* m_entry;				// 0x08
		}; // 0x0C

		INT m_currentGameplayCameraId;		// 0x04
	}; // 0x08

	class ClientGameView
		: public GameView,													// 0x00
		public MessageListener												// 0x08
	{
	public:
		enum FreeCameraMode
		{
			Static,
			Follow,
			LookAt,
		};

		LinearTransform m_cameraTargetOffset;								// 0x10
		Vec3 m_cameraTargetDistance;										// 0x50
		CameraScene* m_freeCameraScene;										// 0x60
		FreeCamera* m_freeCamera;											// 0x64
		FreeCameraMode m_freeCameraMode;									// 0x68
		INT m_freeCameraPriority;											// 0x6C
		PAD(0x8);															// 0x70
		ClientEntryComponent* m_inputTarget;								// 0x78
		EntryComponent::Subscriber* m_inputUnspawnCallback;					// 0x7C
		WeakPtr<ClientGameEntity> m_cameraTarget;							// 0x80
		ClientEntryComponent* m_cameraEntryTarget;							// 0x84
		GameView::EntryUnSpawnCallback* m_targetEntryUnSpawnCallback;		// 0x88
		WeakPtr<ClientCameraContext> m_cameraContext;						// 0x8C
	}; // 0x90

	class CameraScene
	{
	public:
		CameraManager* m_manager;							// 0x00
		eastl::map<UINT, Camera*> m_cameras;				// 0x04
		eastl::map_node<UINT, Camera*>* m_activeCamera;		// 0x1C
	}; // 0x20

	class CameraManager
	{
	public:
		LPVOID vftable;			// 0x00
	}; // 0x04

	class ITypedObjectWithRefCount
		: public ITypedObject			// 0x00
	{
	public:
		INT m_refCount;					// 0x04
	}; // 0x08

	class FovEffect
	{
	public:
		FLOAT m_fov;			// 0x00
		FLOAT m_fadeTime;		// 0x04
		FLOAT m_delay;			// 0x08
		INT m_active;			// 0x0C
	}; // 0x10

	class Camera
		: public ITypedObjectWithRefCount		// 0x00
	{
	public:
		PAD(0x8);								// 0x08
		LinearTransform m_transform;			// 0x10
		Vec2 m_viewportOffset;					// 0x50
		PAD(0x4);								// 0x58
		FovEffect m_fovInEffect;				// 0x5C
		FovEffect m_fovOutEffect;				// 0x6C
		CameraData* m_data;						// 0x7C
		FLOAT m_fov;							// 0x80
		FLOAT m_dofFocusDistance;				// 0x84
	}; // 0x88

	class CameraData
		: public GameObjectData				// 0x00
	{
	public:
		Vec3 m_occlusionRayOffset;			// 0x10
		FLOAT m_shakeFactor;				// 0x20
		FLOAT m_preFadeTime;				// 0x24
		FLOAT m_fadeTime;					// 0x28
		FLOAT m_fadeWaitTime;				// 0x2C
		FLOAT m_soundListenerRadius;		// 0x30
		DataContainer* m_viewFx;			// 0x34
		FLOAT m_nearPlane;					// 0x38
		FLOAT m_soundOcclusion;				// 0x3C
		INT m_stayFadedWhileStreaming;		// 0x40
	}; // 0x44

	class FreeCamera
		: public Camera								// 0x00
	{
	public:
		class State
		{
		public:
			FLOAT rotateLeftRight;		// 0x00
			FLOAT rotateUpDown;			// 0x04
			FLOAT moveLeftRight;		// 0x08
			FLOAT moveUpDown;			// 0x0C
			FLOAT moveReverseForward;	// 0x10
			FLOAT increaseFov;			// 0x14
		}; // 0x18

		PAD(0x8);									// 0x88
		FreeCameraInput* m_currentInput;			// 0x90
		FreeCameraInput* m_defaultCameraInput;		// 0x94
		FreeCameraInput* m_editorCameraInput;		// 0x98
		State m_realState;							// 0x9C
		State m_wantedState;						// 0xB4
		PAD(0x4);									// 0xCC
		Vec3 m_targetPos;							// 0xD0
		FLOAT m_rotateLeftRightHeldDown;			// 0xE0
		FLOAT m_rotateUpDownHeldDown;				// 0xE4
		FLOAT m_moveLeftRightHeldDown;				// 0xE8
		FLOAT m_moveUpDownHeldDown;					// 0xEC
		FLOAT m_moveReverseFowardHeldDown;			// 0xF0
		FLOAT m_increaseFovHeldDown;				// 0xF4
	}; // 0xF8

	class Input
	{
	public:
		LPVOID vftable;				// 0x00
		InputNode* m_inputNode;		// 0x04
	}; // 0x08

	class EntryInputTranslator
		: public Input							// 0x00
	{
	public:
		EntryInputActionMap* m_inputActionMap;	// 0x08
		InputActionMappingsData* m_mappingData;	// 0x0C
	}; // 0x10

	class InputNode
	{
	public:
		LPVOID vftable;			// 0x00
	}; // 0x04

	class FreeCameraInput
		: public Input						// 0x00
	{
	public:
		PAD(0x8);							// 0x08
		Vec3 m_move;						// 0x10
		Vec3 m_rotate;						// 0x20
		DWORD m_simTickCount;				// 0x30
		FLOAT m_inverseTick;				// 0x34
		FLOAT m_fovIncrease;				// 0x38
		InputActionMapping* m_map;			// 0x3C
		INT m_moveSpeed;					// 0x40
		INT m_rotateSpeed;					// 0x44
		CHAR m_hadPostFrame;				// 0x48
		CHAR m_turboSpeed;					// 0x49
		CHAR m_enabled;						// 0x4A
		PAD(0x1);							// 0x4B
	}; // 0x4C

	class InputActionMapping
	{
	public:
		eastl::vector<INT> m_mappedActions;		// 0x00
	}; // 0x10

	class SpatialEntity
		: public Entity				// 0x00
	{
	public:
		DWORD m_cullGridId;			// 0x0C
	}; // 0x10



	class MyEntityMatrix
	{
	public:
		D3DXMATRIX m_transform;			//0x00
	};

	class MyEntity
	{
	public:
		char unknown[0x1C];				//0x00
		MyEntityMatrix *EntityMatrix;	//0x1C
	};

	template <class T>
	class SpatialEntityWithBusAndData
		: public SpatialEntity				// 0x00
	{
	public:
		EntityBus* m_entityBus;				// 0x10
		T* m_data;							// 0x14
	}; // 0x18

	class GameEntity
		: public SpatialEntityWithBusAndData<GameEntityData>	// 0x00
	{
	public:
		DWORD m_updateInterval;									// 0x18
		ComponentCollection* m_collection;						// 0x1C

		template<class T>
		T *getComponent(std::string name)
		{
			if (!m_collection || !((m_flags >> 15) & 1))
				return NULL;
			for (int i = 0; i < m_collection->totalCount; i++)
			{
				fb::ComponentInfo *info = m_collection->getInfo(i);
				if (!info || !info->flags >> 15 || !info->component)
					continue;

				fb::TypeInfo *tinfo = info->component->getType();

				//	LOG.Write("%s",tinfo->m_infoData->name);

				if (!std::string(tinfo->m_infoData->name).compare(name))
					return (T *)info->component;
			}
			return NULL;
		}
	}; // 0x20

	class ClientGameEntity
		: public GameEntity		// 0x00
	{
	public:
		//virtual char Unknown[0xBC]; 0xBC
		//virtual bool VisualUpdate(float deltatime);
	}; // 0x20

	template <class T>
	class GamePhysicsEntity
		: public T				// 0x00
	{
	public:
		FLOAT m_health;			// sizeof(T)
	}; // sizeof(T) + 0x04

	class IRigidBodyHook
	{
	public:
		LPVOID vftable;			// 0x00
	}; // 0x04

	class ClientPhysicsEntity
		: public GamePhysicsEntity<ClientGameEntity>,		// 0x00
		public IRigidBodyHook								// 0x24
	{
	}; // 0x28

	template <class T>
	class ClientGhostGameEntity
		: public T,						// 0x00
		public network::ClientGhost		// sizeof(T)
	{
	}; // sizeof(T) + 0x40

	class ClientGhostAndNetworkableGameEntity
		: public ClientGhostGameEntity<ClientPhysicsEntity>,	// 0x00
		public network::IClientNetworkable						// 0x68
	{
	}; // 0x70

	class ControllableEntity
	{
	public:
		LPVOID vftable;						// 0x00
		MaterialContainerPair* m_material;	// 0x04
		INT m_teamId;						// 0x08
		INT m_defaultTeamId;				// 0x0C
	}; // 0x10

	class ClientControllableEntity
		: public ClientGhostAndNetworkableGameEntity,			// 0x00
		public ControllableEntity								// 0x70
	{
	public:
		__forceinline D3DXVECTOR3* getVehicleSpeed()
		{
			return (D3DXVECTOR3*)((DWORD)this + 0x140);
		}

		eastl::vector<ClientEntryComponent*> m_entries;			// 0x80
		DWORD m_currentVelocityNormalizedOut;					// 0x90 PropertyWriter<FLOAT>
		DWORD m_currentHealthNormalizedOut;						// 0x94 PropertyWriter<FLOAT>
		FLOAT m_oldVelocity;									// 0x98
		FLOAT m_oldHealth;										// 0x9C
		ClientSpawnEntity* m_owner;								// 0xA0
		DWORD m_correctionTask;									// 0xA4
		DWORD m_remoteGhostId;									// 0xA8
		DWORD m_correctingEntry;								// 0xAC
		CHAR m_predictionEnabled;								// 0xB0
		CHAR m_correctionEnabled;								// 0xB1
		CHAR m_isCorrecting;									// 0xB2
		PAD(0x1);												// 0xB3
	}; // 0xB4

	class ReferenceObjectData
		: public GameObjectData					// 0x00
	{
	public:
		LinearTransform m_bluprintTransform;	// 0x10
		DataContainer* m_blueprint;				// 0x50
		DataContainer* m_objectVariation;		// 0x54
		StreamRealm m_streamRealm;				// 0x58
		INT m_excluded;							// 0x5C
	}; // 0x60

	class SpawnReferenceObjectData
		: public ReferenceObjectData
	{
	public:
		LinearTransform m_controllableInput;			// 0x60
		LinearTransform m_controllableTransform;		// 0xA0
		FLOAT m_spawnProtectionRadius;					// 0xE0
		String m_locationNameSid;						// 0xE4
		String m_locationTextSid;						// 0xE8
		INT m_teamId;									// 0xEC
		FLOAT m_rotationRoll;							// 0xF0
		DWORD m_spawnProtectionFriendlyKilledCount;		// 0xF4
		FLOAT m_throttle;								// 0xF8
		FLOAT m_rotationPitch;							// 0xFC
		FLOAT m_initialSpawnDelay;						// 0x100
		FLOAT m_spawnDelay;								// 0x104
		INT m_maxCount;									// 0x108
		INT m_maxCountSimultaneously;					// 0x10C
		INT m_totalCountSimultaneousOfType;				// 0x110
		FLOAT m_spawnAreaRadius;						// 0x114
		FLOAT m_rotationYaw;							// 0x118
		FLOAT m_spawnProtectionFriendlyKilledTime;		// 0x11C
		INT m_takeControlEntryIndex;					// 0x120
		CHAR m_lockedTeam;								// 0x124
		CHAR m_autoSpawn;								// 0x125
		CHAR m_onlySendEventForHumanPlayers;			// 0x126
		CHAR m_clearBangersOnSpawn;						// 0x127
		CHAR m_tryToSpawnOutOfSight;					// 0x128
		CHAR m_sendWeaponEvents;						// 0x129
		CHAR m_takeControlOnTransformChange;			// 0x12A
		CHAR m_returnControlOnIdle;						// 0x12B
		CHAR m_useAsSpawnPoint;							// 0x12C
		CHAR m_initialAutoSpawn;						// 0x12D
		CHAR m_enabled;									// 0x12E
		CHAR m_spawnProtectionCheckAllTeams;			// 0x12F
	};

	class ClientSpawnEntity
		: public SpatialEntityWithBusAndData<SpawnReferenceObjectData>,		// 0x00
		network::ClientGhost,												// 0x18
		network::IClientNetworkable											// 0x58
	{
	public:
		class Interpolator
			: public network::Interpolator<LPVOID>	// 0x00
		{
		public:
			ClientSpawnEntity* m_spawnEntity;		// 0x40
		}; // 0x44

		class ControllableListener
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04

		eastl::vector<ClientControllableEntity*> m_spawnedControllables;	// 0x60
		EntityBus* m_peerSubBus;											// 0x70
		DWORD m_controllableTransform;										// 0x74 PropertyWriter<LinearTransform>
		PAD(0x8);															// 0x78
		D3DXMATRIX m_transform;										// 0x80
		Interpolator m_interpolator;										// 0xC0
		INT m_teamId;														// 0xC4
		INT m_enabled;														// 0xC8
		eastl::vector<ControllableListener> m_controllableListeners;		// 0xCC
	}; // 0xDC

	/*class CharacterEntity
	{
	public:
	LPVOID vftable;		// 0x00
	}; // 0x04
	*/
	class CharacterEntity
	{
	public:
		virtual float yaw(); //
		virtual float pitch(); //
		virtual bool isVisible(); //
		virtual bool isAlive(); //
		virtual bool isDead(); //
		virtual bool isDying(); //
		virtual float getHealth(); //
		virtual float getMaxHealth(); //
		virtual bool isAIPlayer(); //
		virtual bool isSingleplayer(); //
		virtual void Function10(); // void getPhysicsInfoForAnimation
		virtual bool isInVehicle(); //
		virtual void* physics(); // fb::PhysicsEntityBase
		virtual int pose(); // enum fb::CharacterPoseType
		virtual int activeView(); // enum fb::PersonViewEnum

	};//Size=0x0004

	class UpdatePoseResultData
	{
	public:
		ant::QuatTransform* m_localTransforms; //0x0000
		ant::QuatTransform* m_worldTransforms; //0x0004
		__m128* m_renderTransforms; //0x0008
		ant::QuatTransform* m_interpolatedLocalTransforms; //0x000C
		ant::QuatTransform* m_interpolatedWorldTransforms; //0x0010
		ant::QuatTransform* m_activeWorldTransforms; //0x0014
		ant::QuatTransform* m_activeLocalTransforms; //0x0018
		int m_slot; //0x001C
		int m_readerIndex; //0x0020
		bool m_validTransforms; //0x0024
		bool m_poseUpdateEnabled; //0x0025
		bool m_poseNeeded; //0x0026

	};//Size=0x0027
	class Animatable
	{
	public:
		char _0x0000[24];
		UpdatePoseResultData m_updatePoseResultData; //0x0018
		char _0x0040[40];

	};//Size=0x0068

	class SharedPose
	{
	public:
		char _0x0000[64];

	};//Size=0x0040

	class GameAnimatable
	{
	public:
		char _0x0000[196];
		Animatable* m_animatable; //0x00C4
		char _0x00C8[4];
		unsigned int m_animatableInstanceId; //0x00CC
		bool m_hadVisualUpdate; //0x00D0
		char _0x00D1[263];
		SharedPose m_sharedPose; //0x01D8
		char _0x0218[8];
	};//Size=0x0220

	class AntAnimatableComponent
	{
	public:
		char _0x0000[0x20];
		GameAnimatable m_handler; //0x0020
	};

	class ClientAntAnimatableComponent
		:public AntAnimatableComponent
	{
	public:
	};

	class ClientCharacterEntity
		: public ClientControllableEntity,				// 0x00
		public CharacterEntity							// 0xB4
	{
	public:
		ClientPlayer* m_pPlayer;							// 0xB8
		ClientAntAnimatableComponent* m_animatableComponent[2];					// 0xBC
		DWORD m_collisionComponent;						// 0xC4
		DWORD m_healthComponent;						// 0xC8
		WORD m_lightProbeHandle;						// 0xCC
		PAD(0x2);										// 0xCE
		eastl::vector<IInputFilter*> m_inputFilters;	// 0xD0
		DWORD m_hasLocalPlayer;							// 0xE0
		PersonViewEnum m_activeView;					// 0xE4
		Entity** m_ownedEntities;						// 0xE8
		WORD m_ownedEntityCount;						// 0xEC
		CHAR m_mustBeRemotePlayer;						// 0xEE
		PAD(0x1);										// 0xEF
	}; // 0xF0

	class IInputFilter
	{
	public:
		LPVOID vftable;		// 0x00
	}; // 0x04

	class GamePhysicsEntityData
		: public GameEntityData				// 0x00
	{
	public:
		DataContainer* m_physicsData;		// 0x60
	}; // 0x64

	class ControllableEntityData
		: public GamePhysicsEntityData			// 0x00
	{
	public:
		PAD(0x8);								// 0x64
		INT m_defaultTeam;						// 0x6C
		FLOAT m_lowHealthThreshold;				// 0x70
		PAD(0x4);								// 0x74
		DataContainer* m_materialPair;			// 0x78
		PAD(0x4);								// 0x7C
	}; // 0x80

	class CharacterEntityData
		: public ControllableEntityData			// 0x00
	{
	public:
		PlayerSpawnType m_pPlayerSpawnType;		// 0x80
		PersonViewMode m_defaultViewMode;		// 0x84
		PAD(8);		// 0x88
	};

	class SoldierEntityData
		: public CharacterEntityData			// 0x00
	{
	public:
		PAD(0x10);								// 0x90
		Vec3 m_fLIRKeyColor; 					// 0xA0
		DataContainer* m_headMaterialPair; 		// 0xB0
		DataContainer* m_boneFakePhysics; 		// 0xB4
		DataContainer* m_autoAim; 				// 0xB8
		DataContainer* m_aimingConstraints; 	// 0xBC
		DataContainer* m_headCollision; 		// 0xC0
		DataContainer* m_characterPhysics; 		// 0xC4
		DataContainer* m_footMaterialPair; 		// 0xC8
		DataContainer* m_sprintSettings; 		// 0xCC
		DataContainer* m_meshes1p; 				// 0xD0
		DataContainer* m_meshes3p; 				// 0xD4
		DataContainer* m_hidableMeshParts; 		// 0xD8
		DataContainer* m_sound; 				// 0xDC
		DataContainer* m_streamGroup1p; 		// 0xE0
		FLOAT m_impulseReactionTime; 			// 0xE4
		DataContainer* m_healthModule; 			// 0xE8
		FLOAT m_maxHealth; 						// 0xEC
		DataContainer* m_collisionInfo; 		// 0xF0
		DataContainer* m_voiceOverInfo; 		// 0xF4
		FLOAT m_fLIRValue; 						// 0xF8
		DataContainer* m_breathControl; 		// 0xFC
		FLOAT m_explosionDamageModifier; 		// 0x100
		DataContainer* m_uiParts; 				// 0x104
		FLOAT m_regenerationDelayModifier; 		// 0x108
		CHAR m_freeSpaceCheck; 					// 0x10C
		CHAR m_proximityCheck; 					// 0x10D
		CHAR m_enableGroundmapLighting; 		// 0x10E
		CHAR m_showWeaponWhenDead; 				// 0x10F
		CHAR m_lowerGunOnOwnTeam; 				// 0x110
		CHAR m_useSpineXRotation; 				// 0x111
		CHAR m_collisionEnabled; 				// 0x112
		CHAR m_physicsControlled; 				// 0x113
		CHAR m_humanPlayerControlled; 			// 0x114
		CHAR m_interactiveManDownAllowed; 		// 0x115
		CHAR m_showNametag; 					// 0x116
	};

	class AutoAimData
	{
	public:
		Vec3 m_autoAimOuterBoxOffset;		 // 0x00
		Vec3 m_autoAimOuterBoxExtends;		 // 0x10
		Vec3 m_autoAimInnerBoxOffset;		 // 0x20
		Vec3 m_autoAimInnerBoxExtends;		 // 0x30
		CharacterPoseType	m_poseType;		 // 0x40
	};

	class SupportedShootingCallback
	{
	public:
		LPVOID vftable;			// 0x00
	}; // 0x04

	class EyePositionCallback
	{
	public:
		LPVOID vftable;			// 0x00
	}; // 0x04

	class PhysicsEntityUserData
	{
	public:
		DWORD mmaterialIndices;		// 0x00
		DWORD materialFlags;		// 0x04
		DWORD partsEnabled;			// 0x08
		DWORD flags;				// 0x0C
		FLOAT mass;					// 0x10
		WORD partCount;				// 0x14
		BYTE materialCount;			// 0x16
		BYTE pad;					// 0x17
	}; // 0x18

	class PhysicsEntityBase
		: public SpatialEntity						// 0x00
	{
	public:
		PhysicsEntityUserData m_physicsUserData;	// 0x10
		PAD(0x8);									// 0x28
		ITypedObject* m_userData;					// 0x30
		DWORD m_em;									// 0x34 SpatialQueryManager
		HavokPhysicsManager* m_manager;				// 0x38 HavokPhysicsManager
		DWORD* m_hook;								// 0x3C HavokRigidBodyHook
		PAD(0x8);									// 0x40
		CHAR m_isAddedToEntityGrid;					// 0x48
		PAD(0xF);									// 0x49
	}; // 0x58

	/*class CharacterPhysicsEntity
		: public PhysicsEntityBase									// 0x00
	{
	public:
		class CharacterInput
		{
		public:
			Vec3 m_forwardVector;		// 0x00
			Vec3 m_upVector;			// 0x10
			Vec3 m_characterGravity;	// 0x20
			FLOAT m_waterLevel;			// 0x30
			FLOAT m_speedScale;			// 0x34
			FLOAT m_yaw;				// 0x38
			FLOAT m_pitch;				// 0x3C
			FLOAT m_forwardInput;		// 0x40
			FLOAT m_strafeInput;		// 0x44
			FLOAT m_sprintMultiplier;	// 0x48
			CHAR m_tryJump;				// 0x4C
			CHAR m_changePose;			// 0x4D
			CHAR m_toggleParachute;		// 0x4E
			CHAR m_sprint;				// 0x4F
			LPVOID m_userData;			// 0x50
		}; // 0x54

		class CharacterPhysicsEntityPos
		{
		public:
			WeakPtr<CharacterPhysicsEntity> entity;		// 0x00
			PAD(0xC);									// 0x04
			Vec3 pos;									// 0x10
		}; // 0x20

		PAD(0x8);						// 0x58
		hkpCharacterProxy* m_characterProxy;						// 0x60
		CharacterPhysicsEntityContext* m_characterContext;			// 0x64
		DWORD m_stateManager;										// 0x68
		DWORD m_phantom;											// 0x6C
		LinearTransform* m_gameWorldTransform;						// 0x70
		DWORD m_listener;											// 0x74
		PAD(0x4);													// 0x78
		CharacterPoseType m_currentPose;							// 0x7C
		CharacterPoseType m_changingToPose;							// 0x80
		FLOAT m_poseTransitionTimer;								// 0x84
		CharacterPhysicsEntityCollisionShapes* m_collisionShapes;	// 0x88
		CharacterPoseConstraints* m_poseConstraints;				// 0x8C
		CHAR m_enabled;												// 0x90
		CHAR m_isPhysicsControlled;									// 0x91
		CHAR m_aiBodyAddedToWorld;									// 0x92
		PAD(0xD);													// 0x93
		Vec3 m_currentLocalEyePosition;								// 0xA0
		FLOAT m_mass;												// 0xB0
		CharacterPhysicsData* m_data;								// 0xB4
		eastl::fixed_vector<CharacterPoseData*, 3, 2> m_poseData;	// 0xB8
		PAD(0x8);													// 0xD0
		EntityBusPeer* m_transformNode;								// 0xD8
		DWORD m_rbProxy;											// 0xDC
		DWORD m_aiBody;												// 0xE0
		DWORD m_contactMaterialIndex;								// 0xE4
		FLOAT m_dynamicFriction;									// 0xE8
		FLOAT m_staticFriction;										// 0xEC
		MaterialContainerPair* m_characterMaterials;				// 0xF0
		DWORD m_characterMaterialFlags;								// 0xF4
		PAD(0x8);													// 0xF8
		Vec3 m_ladderPosition;										// 0x100
		PAD(0x10);													// 0x110
		CharacterInput m_characterInput;							// 0x120
		PAD(0x1C);													// 0x174
		CharacterPhysicsEntityPos m_adjustedEntity[4];				// 0x190
		DWORD m_adjustCount;										// 0x210
	}; // 0x214*/

	class CharacterPhysicsEntity
		: public PhysicsEntityBase									// 0x00
	{
	public:
		class CharacterInput
		{
		public:
			Vec3 m_forwardVector;		// 0x00
			Vec3 m_upVector;			// 0x10
			Vec3 m_characterGravity;	// 0x20
			FLOAT m_waterLevel;			// 0x30
			FLOAT m_speedScale;			// 0x34
			FLOAT m_yaw;				// 0x38
			FLOAT m_pitch;				// 0x3C
			FLOAT m_forwardInput;		// 0x40
			FLOAT m_strafeInput;		// 0x44
			FLOAT m_sprintMultiplier;	// 0x48
			CHAR m_tryJump;				// 0x4C
			CHAR m_changePose;			// 0x4D
			CHAR m_toggleParachute;		// 0x4E
			CHAR m_sprint;				// 0x4F
			LPVOID m_userData;			// 0x50
		}; // 0x54

		class CharacterPhysicsEntityPos
		{
		public:
			WeakPtr<CharacterPhysicsEntity> entity;		// 0x00
			PAD(0xC);									// 0x04
			Vec3 pos;									// 0x10
		}; // 0x20

		class OrientationCallback
		{
		public:
			virtual void setYaw(float);	// V: 0x0
			virtual float getYaw();	// V: 0x4
			virtual void setPitch(float);	// V: 0x8
			virtual float getPitch();	// V: 0xC

		}; // OrientationCallback

		PAD(0x8);						// 0x58
		hkpCharacterProxy* m_characterProxy;						// 0x60
		CharacterPhysicsEntityContext* m_characterContext;			// 0x64
		DWORD m_stateManager;										// 0x68
		DWORD m_phantom;											// 0x6C
		LinearTransform* m_gameWorldTransform;						// 0x70
		DWORD m_listener;											// 0x74
		PAD(0x4);													// 0x78
		CharacterPoseType m_currentPose;							// 0x7C
		CharacterPoseType m_changingToPose;							// 0x80
		FLOAT m_poseTransitionTimer;								// 0x84
		CharacterPhysicsEntityCollisionShapes* m_collisionShapes;	// 0x88
		CharacterPoseConstraints* m_poseConstraints;				// 0x8C
		CHAR m_enabled;												// 0x90
		CHAR m_isPhysicsControlled;									// 0x91
		CHAR m_aiBodyAddedToWorld;									// 0x92
		PAD(0xD);													// 0x93
		Vec3 m_currentLocalEyePosition;								// 0xA0
		FLOAT m_mass;												// 0xB0
		CharacterPhysicsData* m_data;								// 0xB4
		eastl::fixed_vector<CharacterPoseData*, 3, 2> m_poseData;	// 0xB8
		PAD(0x8);													// 0xD0
		EntityBusPeer* m_transformNode;								// 0xD8
		DWORD m_rbProxy;											// 0xDC
		DWORD m_aiBody;												// 0xE0
		DWORD m_contactMaterialIndex;								// 0xE4
		FLOAT m_dynamicFriction;									// 0xE8
		FLOAT m_staticFriction;										// 0xEC
		MaterialContainerPair* m_characterMaterials;				// 0xF0
		DWORD m_characterMaterialFlags;								// 0xF4
		PAD(0x8);													// 0xF8
		Vec3 m_ladderPosition;										// 0x100
		PAD(0x10);													// 0x110
		CharacterInput m_characterInput;							// 0x120
		PAD(0x1C);													// 0x174
		CharacterPhysicsEntityPos m_adjustedEntity[4];				// 0x190
		DWORD m_adjustCount;										// 0x210
		void getLocalBoundingBox(fb::AxisAlignedBox* out)
		{
			typedef void(__thiscall * R_getLocalBox)(fb::CharacterPhysicsEntity* pThis, fb::AxisAlignedBox* out);
			R_getLocalBox m_getLocalBox = (R_getLocalBox)0x5801A0;

			return m_getLocalBox(this, out);
		}

		void getLocalBoundingBoxForPose(AxisAlignedBox* out, /*enum  fb::CharacterPoseType*/int pose, bool expandWithVisualRepresentation = true)
		{

			typedef void(__thiscall * R_getLocalBoxForPose)(fb::CharacterPhysicsEntity* pThis, fb::AxisAlignedBox* out,/*enum  fb::CharacterPoseType*/ int pose, bool expandWithVisualRepresentation);
			R_getLocalBoxForPose m_getLocalBoxForPose = (R_getLocalBoxForPose)0x00589D40;

			return m_getLocalBoxForPose(this, out, pose, expandWithVisualRepresentation);
		}


	}; // 0x214

	//class SoldierEntity
	//{
	//public:
	//	//virtual class fb::BoneCollisionComponent * boneCollisionComponent();	// V: 0x0
	//	//virtual const struct fb::LinearTransform & soldierTransform();	// V: 0x4
	//	//virtual bool isManDown();	// V: 0x8
	//	//virtual bool isInteractiveManDown();	// V: 0xC
	//	//virtual bool hasRestrictedMovement();	// V: 0x10
	//	//virtual bool isFiring();	// V: 0x14
	//	//virtual bool isReloading();	// V: 0x18
	//	//virtual float getManDownTimeLeft();	// V: 0x1C
	//	//virtual const class fb::WeaponSway * getWeaponSway();	// V: 0x20
	//	//virtual const class fb::WeaponFiring * getCurrentWeaponFiring();	// V: 0x24
	//	//virtual const class fb::WeaponFiringData * getCurrentWeaponFiringData();	// V: 0x28
	//	//virtual float moveSpeedMultiplier();	// V: 0x2C
	//	LPVOID vftable;												// 0x00
	//	PAD(0xC);													// 0x04
	//	SoldierEntityData* m_soldierData;							// 0x10
	//	AutoAimData* m_autoAimData[3];								// 0x14
	//	INT m_overrideLookAt;										// 0x20
	//	PAD(0xC);													// 0x24
	//	Vec3 m_lookTarget;											// 0x30
	//	FLOAT m_headRotationSpeed;									// 0x40
	//	INT m_allowedToMoveEyes;									// 0x44
	//	FLOAT m_waterLevelUpdateTimer;								// 0x48
	//	PAD(0x4);													// 0x4C
	//	Vec3 m_hitDirection;										// 0x50
	//	MaterialContainerPair* m_headMaterial;						// 0x60
	//	Component* m_space;											// 0x64
	//	SupportedShootingCallback* m_supportedShootingCallback;		// 0x68
	//	EyePositionCallback* m_eyePositionCallback;					// 0x6C
	//	FLOAT m_maxHealth;											// 0x70
	//	CharacterPhysicsEntity* m_characterPhysicsentity;			// 0x74
	//	PAD(0x8);													// 0x78
	//	CharacterPhysicsEntity::CharacterInput m_characterInput;	// 0x80
	//	PAD(0xC);													// 0xD4
	//	INT m_beingInteractedStatus;								// 0xE0
	//	FLOAT m_waterLevel;											// 0xE4
	//	CHAR m_entryPoseEnabled;									// 0xE8
	//	CHAR m_physicsControlled;									// 0xE9
	//	CHAR m_isShielded;											// 0xEA
	//	PAD(0x1);													// 0xEB
	//	CharacterEntity* m_character;								// 0xEC
	//}; // 0xF0
	class QuatTransform
	{
	public:
		// use Vec3 and skip the reference to quat which is also only a vec
		//		Vec4 transAndScale;                     // 0x0
		//		Quat rotation;                     // 0x10*/
		Vec3 transAndScale;		// 0x00
		Vec3 rotation;			// 0x10
	}; // fb::QuatTransform

	/*class SoldierEntity
	{
	public:
		virtual void Function1();
		virtual fb::LinearTransform soldierTransform(); //
		virtual bool isManDown(); //
		virtual bool isInteractiveManDown(); //
		virtual bool hasRestrictedMovement(); //
		virtual bool isFiring(); //
		virtual bool isReloading(); //
		virtual float getManDownTimeLeft(); //
		virtual fb::WeaponSway* getWeaponSway(); //
		virtual fb::WeaponFiring* getCurrentWeaponFiring(); //
		virtual fb::WeaponFiringData* getCurrentWeaponFiringData(); //
		virtual float moveSpeedMultiplier(); //
		virtual void Function12(); //
		virtual void Function13(); //
		virtual void Function14(); //
		virtual void Function15(); //
		virtual void Function16(); //
		virtual void Function17(); //
		virtual void Function18(); //
		virtual void Function19(); //
		virtual void Function20(); //
		virtual void Function21(); //
		virtual void Function22(); //
		virtual void Function23(); //
		virtual void Function24(); //
		virtual void Function25(); //

		char _0x0004[108];
		float m_maxHealth; //0x0070
		char _0x0074[120];
		CharacterEntity* m_character; //0x00EC

	};//Size=0x0218*/

	class SoldierEntity   // Inherited class at offset 0x170
	{
	public:
		enum DeathType
		{
			Shot,
			ShotInVehicleEntry,
			InsideExplodingVehicle,
			DeathTypeCount,
		};

		enum SoldierInteractedStatus
		{
			SoldierInteractedStatus_None,
			SoldierInteractedStatus_BeingInteracted,
			SoldierInteractedStatus_BeingInteractedCancelled,
			SoldierInteractedStatus_BeingInteractedFinished,
			SoldierInteractedStatus_Count,
		};

		virtual BoneCollisionComponent * boneCollisionComponent();	// V: 0x0
		virtual LinearTransform & soldierTransform();	// V: 0x4
		virtual bool isManDown();	// V: 0x8
		virtual bool isInteractiveManDown();	// V: 0xC
		virtual bool hasRestrictedMovement();	// V: 0x10
		virtual bool isFiring();	// V: 0x14
		virtual bool isReloading();	// V: 0x18
		virtual void funkyme();
		virtual float getManDownTimeLeft();	// V: 0x1C
		virtual WeaponSway * getWeaponSway();	// V: 0x20
		virtual  WeaponFiring * getCurrentWeaponFiring();	// V: 0x24
		virtual WeaponFiringData * getCurrentWeaponFiringData();	// V: 0x28
		virtual float moveSpeedMultiplier();	// V: 0x2C
		virtual void* boundsChecker();	// V: 0x30
		virtual DeathType deathType();	// V: 0x34
		virtual SoldierEntityActionState actionState();	// V: 0x40
		/*
		virtual void Function15(); //
		virtual void Function16(); //
		virtual void Function17(); //
		virtual void Function18(); //
		virtual void Function19(); //
		virtual void Function20(); //
		virtual void Function21(); //
		virtual DWORD computeBoundingBoxWorldTransform( fb::LinearTransform* matrix ); //
		virtual DWORD computeBoundingBox( fb::AxisAlignedBox* aabb ); //
		virtual void Function25(); //
		*/
		PAD(0xC);													// 0x04
		SoldierEntityData* m_soldierData;							// 0x10



		AutoAimData* m_autoAimData[3];								// 0x14
		//changes
		bool m_overrideLookAt;										// 0x20
		PAD(0xC);													// 0x24
		Vec3 m_lookTarget;											// 0x30
		FLOAT m_headRotationSpeed;									// 0x40
		INT m_allowedToMoveEyes;									// 0x44
		FLOAT m_waterLevelUpdateTimer;								// 0x48
		PAD(0x4);													// 0x4C
		Vec3 m_hitDirection;										// 0x50
		MaterialContainerPair* m_headMaterial;						// 0x60
		Component* m_space;											// 0x64
		SupportedShootingCallback* m_supportedShootingCallback;		// 0x68
		EyePositionCallback* m_eyePositionCallback;					// 0x6C
		FLOAT m_maxHealth;											// 0x70
		CharacterPhysicsEntity* m_characterPhysicsentity;			// 0x74
		PAD(0x8);													// 0x78
		CharacterPhysicsEntity::CharacterInput m_characterInput;	// 0x80
		PAD(0xC);													// 0xD4
		INT m_beingInteractedStatus;								// 0xE0
		FLOAT m_waterLevel;											// 0xE4
		CHAR m_entryPoseEnabled;									// 0xE8
		CHAR m_physicsControlled;									// 0xE9
		CHAR m_isShielded;											// 0xEA
		PAD(0x1);													// 0xEB
		CharacterEntity* m_character;								// 0xEC


	}; // 0xF0

	class CharacterPhysicsEntityContext
		: public hkpCharacterContext							// 0x00
	{
	public:
		class ParachuteContext
		{
		public:
			LinearTransform m_transformWS;		// 0x00
			LinearTransform m_unknown;			// 0x40
			Vec3 m_parachuteOffset;				// 0x80
			FLOAT m_time;						// 0x90
			INT m_initialized;					// 0x94
		}; // 0x98

		PAD(0xC);												// 0x24
		ParachuteContext m_parachuteContext;					// 0x30
		PAD(0x8);												// 0xC8
		Vec3 m_ladderNorm;										// 0xD0
		CharacterPhysicsEntityCallbacks* m_callbackHandler;		// 0xE0
		CharacterPhysicsEntity* m_entity;						// 0xE4
		DWORD m_ladder;											// 0xE8
		FLOAT m_timeToJump;										// 0xEC
		FLOAT m_jumpDelay;										// 0xF0
		FLOAT m_stamina;										// 0xF4
		FLOAT m_jumpPenaltyFactor;								// 0xF8
		FLOAT m_forceToGroundTimer;								// 0xFC
		DWORD m_jumpCount;										// 0x100
		CHAR m_applyLandingPenalty;								// 0x104
		CHAR m_sprinting;										// 0x105
		CHAR m_sprintHold;										// 0x106
		CHAR m_jumpInProgress;									// 0x107
	};

	class CharacterPhysicsEntityCallbacks
	{
	public:
		LPVOID vftable;				// 0x00
	}; // 0x04

	class AxisAlignedBox
	{
	public:
		Vec3 min;			// 0x00
		Vec3 max;			// 0x10
	}; // 0x20

	class CharacterPhysicsEntityCollisionShapes
	{
	public:
		hkpShape* m_shapes[3];
		PAD(0xC);
		AxisAlignedBox m_aabbs[3];
	};

	class CharacterPoseConstraints
	{
	public:
		LPVOID vftable;			// 0x00
		CHAR m_validPoses[4];	// 0x04
	}; // 0x08

	class CharacterPhysicsData
		: public Asset								// 0x00
	{
	public:
		RefArray<CharacterPoseData> m_poses;		// 0x0C
		RefArray<CharacterStateData> m_states;		// 0x10
		CharacterStateType m_defaultState;			// 0x14
		DataContainer* m_sprint;					// 0x18
		DataContainer* m_materialPair;				// 0x1C
		INT m_pushableObjectWeight;					// 0x20
		FLOAT m_mass;								// 0x24
		FLOAT m_maxAscendAngle;						// 0x28
		FLOAT m_physicalRadius;						// 0x2C
		FLOAT m_waterDepthLimit;					// 0x30
		FLOAT m_inputAcceleration;					// 0x34
		FLOAT m_ladderAcceptAngle;					// 0x38
		FLOAT m_ladderAcceptAnglePitch;				// 0x3C
		FLOAT m_jumpPenaltyTime;					// 0x40
		FLOAT m_jumpPenaltyFactor;					// 0x44
		FLOAT m_speedPenaltyOnDamage;				// 0x48
	}; // 0x4C

	class LookConstraintsData
	{
	public:
		FLOAT m_minLookYaw;			// 0x00
		FLOAT m_maxLookYaw;			// 0x04
		FLOAT m_minLookPitch;		// 0x08
		FLOAT m_maxLookPitch;		// 0x0C
	}; // 0x10

	class CharacterPoseData
		: public DataContainer							// 0x00
	{
	public:
		PAD(0x8);										// 0x08
		Vec3 m_eyePosition;								// 0x10
		Vec3 m_collisionBoxMaxExpand;					// 0x20
		Vec3 m_collisionBoxMinExpand;					// 0x30
		FLOAT m_height;									// 0x40
		FLOAT m_stepHeight;								// 0x44
		Array<Vec2> m_throttleModifierCurve;			// 0x48
		CharacterPoseType m_poseType;					// 0x4C
		CharacterPoseCollisionType m_collisionType;		// 0x50
		LookConstraintsData m_lookConstraints;			// 0x54
	}; // 0x64

	class CharacterStateData
		: public DataContainer							// 0x00
	{
	public:
		RefArray<CharacterStatePoseInfo> m_poseInfo;	// 0x08
	}; // 0x0C

	class SpeedModifierData
	{
	public:
		FLOAT m_forwardConstant;		// 0x00
		FLOAT m_backwardConstant;		// 0x04
		FLOAT m_leftConstant;			// 0x08
		FLOAT m_rightConstant;			// 0x0C
	}; // 0x10

	class CharacterStatePoseInfo
		: public DataContainer				// 0x00
	{
	public:
		CharacterPoseType m_poseType;		// 0x08
		FLOAT m_velocity;					// 0x0C
		FLOAT m_accelerationGain;			// 0x10
		FLOAT m_decelerationGain;			// 0x14
		FLOAT m_sprintGain;					// 0x18
		FLOAT m_sprintMultiplier;			// 0x1C
		SpeedModifierData m_speedModifier;	// 0x20
	};	// 0x30

	class ClientSoldierEntity
		: public
		ClientCharacterEntity,									// 0x00
		public SoldierEntity,											// 0xF0
		public network::Interpolator<LPVOID>,							// 0x1E0
		public CharacterPhysicsEntityCallbacks							// 0x220
	{
	public:
		virtual TypeInfo* getType();
		virtual void Function1(); //
		virtual void Function2(); //
		virtual void Function3(); //
		virtual void Function4(); //
		virtual void Function5(); //
		virtual void Function6(); //
		virtual void Function7(); //
		virtual void Function8(); //
		virtual void Function9(); //
		virtual void Function10(); //
		virtual void Function11(); //
		virtual void Function12(); //
		virtual void Function13(); //
		virtual void Function14(); //
		virtual void Function15(); //
		virtual void Function16(); //
		virtual void Function17(); //
		virtual void Function18(); //
		virtual void Function19(); //
		virtual void getTransform(fb::LinearTransform*);//20 //virtual void getTransform(fb::LinearTransform*);//20
		virtual void setTransform(fb::LinearTransform*);//21
		virtual fb::LinearTransform* computeBoundingBoxWorldTransform(fb::LinearTransform* matrix); //22
		virtual fb::AxisAlignedBox* computeBoundingBox(fb::AxisAlignedBox* aabb); //23
		virtual void Function24(); //
		virtual void Function25(); //
		virtual void Function26(); //
		virtual void spawn(fb::LinearTransform*, void*, bool); //
		virtual void Function28(); //
		virtual void Function29(); //
		virtual void Function30(); //
		virtual void Function31(); //
		virtual void Function32(); //
		virtual void Function33(); //
		virtual void Function34(); //
		virtual void Function35(); //
		virtual void Function36(); //
		virtual void Function37(); //
		virtual void Function38(); //
		virtual void Function39(); //
		virtual void Function40(); //
		virtual void Function41(); //
		virtual void Function42(); //
		virtual void Function43(); //
		virtual void Function44(); //
		virtual void Function45(); //
		virtual void Function46(); //
		virtual void Function47(); //
		virtual void Function48(); //
		virtual void Function49(); //
		virtual void Function50(); //
		virtual void Function51(); //
		virtual void Function52(); //
		virtual void Function53(); //
		virtual fb::Vec3* velocity(); //
		virtual void Function55(); //
		virtual void Function56(); //
		virtual void Function57(); //
		virtual void Function58(); //
		virtual void Function59(); //
		virtual void Function60(); //
		virtual void Function61(); //
		virtual void Function62(); //
		virtual void Function63(); //
		virtual void Function64(); //
		virtual void Function65(); //
		virtual float getHealth(); //
		virtual float getMaxHealth(); //
		virtual void Function68(); //
		virtual void Function69(); //
		virtual void Function70(); //
		virtual void Function71(); //
		virtual void Function72(); //
		virtual void Function73(); //

		class PersonView
		{
		public:
			eastl::vector<DWORD> m_meshModels;		// 0x00 MeshModel*
			DWORD m_weaponBoneIndex;				// 0x10
		}; // 0x14

		class BreathControlHandler
		{
		public:
			BreathControlData* m_data;					// 0x00
			FLOAT m_breathControlTimer;					// 0x04
			FLOAT m_breathControlMultiplier;			// 0x08
			FLOAT m_breathControlPenaltyTimer;			// 0x0C
			FLOAT m_breathControlPenaltyMultiplier;		// 0x10
			BYTE m_breathControlActive;					// 0x14 Lower two bits set if active
			PAD(0x3);									// 0x15
		}; // 0x18


		class SprintInputHandler
		{
		public:
			enum State
			{
				WaitForward,
				WaitReleaseForward,
				DoubleTapSprint,
				ButtonSprint,
				WaitReleaseButtonSprint,
				PostSprint,
			};

			State m_currentState;			// 0x00
			FLOAT m_doubleTapTimer;			// 0x04
			FLOAT m_sprintReleaseTimer;		// 0x08
			INT m_waitForSprintRelease;		// 0x0C
		}; // 0x10

		PAD(0x20);														// 0x224
		Blueprint* m_blueprint;											// 0x234
		ClientSoldierPrediction* m_predictedController;					// 0x238
		ClientSoldierReplication* m_replicatedController;				// 0x23C
		DWORD m_vegetationInfo;											// 0x240
		PAD(0x4);														// 0x244
		FLOAT m_movementPenalty;										// 0x248
		FLOAT m_clearMovementTimer;										// 0x24C
		DWORD m_customizationComponent;									// 0x250
		INT m_received1pHint;											// 0x254
		DWORD m_cameraComponent;										// 0x258
		MaterialContainerPair* m_footMaterial;							// 0x25C
		PersonView m_personViews[2];									// 0x260
		eastl::vector<DWORD> m_hiddenBones;								// 0x288
		FLOAT m_authorativeYaw;											// 0x298
		PAD(0x4);														// 0x29C
		INT m_aimingenabled;											// 0x2A0
		FLOAT m_authorativePitch;										// 0x2A4
		DWORD m_soldierSound;											// 0x2A8
		PAD(0x4);														// 0x2AC
		D3DXMATRIX m_meshTransform;										// 0x2B0
		SoldierEntityActionState m_oldActionState;						// 0x2F0
		CharacterPoseType m_previousPose;								// 0x2F4
		PAD(0x3C);														// 0x2F8
		DWORD m_proximityHook;											// 0x334
		DWORD m_orientationCallback;									// 0x338
		PAD(0x8);														// 0x33C
		IClientSoldierHealthModule* m_healthModule;						// 0x344
		FLOAT m_deathTimer;												// 0x348
		ClientSoldierWeaponsComponent* m_soldierWeaponsComponent;		// 0x34C
		DWORD m_bodyComponent;											// 0x350
		ClientBoneCollisionComponent* m_boneCollisionComponent;			// 0x35
		PAD(0x8);														// 0x358
		fb::BreathControlHandler* m_breathControlHandler;					// 0x360
		SprintInputHandler* m_sprintInputHandler;						// 0x364
		EntryInputActionEnum m_sprintInterruptAction;					// 0x368
		FLOAT m_sprintRecoveryTimer;									// 0x36C
		CHAR m_wasSprinting;											// 0x370
		CHAR m_isOccluded;												// 0x371
		PAD(0x6);														// 0x372
		FLOAT m_criticalHealthThreshold;								// 0x378
	};


	class ClientSoldierSimulation
	{
	public:
		LPVOID vftable;									// 0x00
		CharacterPhysicsEntity* m_characterEntity;		// 0x04
	}; // 0x08

	class CharacterPhysicsEntityState
	{
	public:
		Vec3 m_parachuteRotation;				// 0x00
		FLOAT m_parachuteDeployTime;			// 0x10
		PAD(0xC);								// 0x14
		Vec3 m_position;						// 0x20
		Vec3 m_surfaceVelocity;					// 0x30
		Vec3 m_linearVelocity;					// 0x40
		Vec3 m_groundNormal;					// 0x50
		DWORD m_pose;							// 0x60
		DWORD m_changingToPose;					// 0x64
		FLOAT m_transitionTimer;				// 0x68
		DWORD m_currentState;					// 0x6C
		BYTE m_enabled;							// 0x70
		PAD(0xF);								// 0x71
		Vec3 m_localEyePosition;				// 0x80
		DWORD m_groundSupported;				// 0x90
		FLOAT m_groundHeight;					// 0x94
		FLOAT m_timeToJump;						// 0x98
		FLOAT m_stamina;						// 0x9C
		FLOAT m_ladderHeightClimbed;			// 0xA0
		FLOAT m_ladderTransitionProgress;		// 0xA4
		PAD(0x44);								// 0xA8
		DWORD m_contactMaterialIndex;			// 0xEC
		FLOAT m_jumpPenaltyFactor;				// 0xF0
		DWORD m_jumpCounter;					// 0xF4
		FLOAT m_forceToGroundTimer;				// 0xF8
		CHAR m_jumpInProgress;					// 0xFC
		CHAR m_applyLandingPenalty;				// 0xFD
		CHAR m_sprinting;						// 0xFE
		CHAR m_sprintHold;						// 0xFF
	}; // 0x100

	class ClientSoldierPrediction
		: public ClientSoldierSimulation,				// 0x00
		public IInputFilter								// 0x08
	{
	public:
		class DeltaState
		{
		public:
			Vec3 position;		// 0x00
			Vec3 velocity;		// 0x10
			Vec3 eyePosition;	// 0x20
		}; // 0x30

		PAD(0x4);										// 0x0C
		DeltaState m_correctionDelta;					// 0x10
		DeltaState m_frameCorrectionDelta;				// 0x40
		Vec3 m_velocity;								// 0x70
		FLOAT m_correctionInterpolationTime;			// 0x80
		FLOAT m_correctionInterpolationTimer;			// 0x84
		FLOAT m_frameInterpolationFactor;				// 0x88
		INT m_noInterpolateNextCorrection;				// 0x8C
		CharacterPhysicsEntityState* m_currentState;	// 0x90
		PAD(0xC);										// 0x94
		CharacterPhysicsEntityState m_predictionState;	// 0xA0
		CharacterPhysicsEntityState m_correctionState;	// 0x1A0
		SoldierEntityActionState m_actionState;			// 0x2A0
	}; // 0x2A4

	class ClientSoldierReplication
		: public ClientSoldierSimulation	// 0x00
	{
	public:
		class State
		{
		public:
			Vec3 parachuteRotation;			// 0x00
			Vec3 position;					// 0x10
			Vec3 surfaceVelocity;			// 0x20
			Vec3 velocity;					// 0x30 //Playerspeed
			Vec3 groundNormal;				// 0x40
			DWORD groundSupported;			// 0x50
			DWORD groundMaterialIndex;		// 0x54
			INT state;						// 0x58
			INT pose;						// 0x5C
			INT changingToPose;				// 0x60
			PAD(0x4);						// 0x64
			FLOAT groundHeight;				// 0x68
			FLOAT ladderHeightClimbed;		// 0x6C
			FLOAT ladderHeightLeft;			// 0x70
			PAD(0x4);						// 0x74
			INT actionState;				// 0x78
		}; // 0x7C

		DWORD m_interpolator;			// 0x08
		PAD(0x4);						// 0x0C
		State m_state;					// 0x10
	}; // 0x8C

	class IClientSoldierHealthModule
	{
	public:
		LPVOID vftable;		// 0x00
	}; // 0x04

	class BoneCollisionComponent
	{
	public:
		enum Bone
		{
			Bone_RightShoulder,
			Bone_LeftShoulder,
			Bone_Pelvis,
			Bone_Head,
			Bone_RightThigh,
			Bone_LeftThigh,
			Bone_RightKnee,
			Bone_LeftKnee,
			Bone_Total
		};

		class BoneTransformInfo
		{
		public:
			///D3DXMATRIX transform;		// 0x00
			LinearTransform transform;      // 0x00
			Vec3 position;					// 0x40
		}; // 0x50

		BoneCollisionComponentData* m_boneCollisionData;							// 0x00
		ant::UpdatePoseResultData m_updatePoseResultData;							// 0x04
		ant::AnimationSkeleton* m_skeleton;											// 0x2C
		BoneTransformInfo* m_boneCollisionTransforms;								// 0x30
		eastl::vector<eastl::pair<INT, MaterialContainerPair>> m_boneCollisionInfo;	// 0x34
		FLOAT m_latencyBufferTime;													// 0x44
		FLOAT m_latencyBufferInterval;												// 0x48
		PAD(0x174);																	// 0x4C
		BoneTransformInfo* m_latencyTransforms;										// 0x1C0
		CHAR m_hiLod;																// 0x1C4
		PAD(0x3);																	// 0x1C5
		DWORD m_debugColor;															// 0x1C8
		INT m_collisionBoneCount;													// 0x1CC
		CHAR m_collisionEnabled;													// 0x1D0
		CHAR m_collisionUpdated;													// 0x1D1
		CHAR m_isServer;															// 0x1D2
		PAD(0x1);																	// 0x1D3
	}; // 0x1D4

	class BoneCollisionComponentData
		: public ComponentData								// 0x00
	{
	public:
		PAD(0x8);											// 0x58
		SkeletonCollisionData* m_skeletonCollisionData;		// 0x60
	}; // 0x64

	class SkeletonCollisionData
		: public DataContainer								// 0x00
	{
	public:
		SkeletonAsset* m_skeletonAsset;						// 0x08
		Array<BoneCollisionData> m_boneCollisionTransforms; // 0x0C
	};

	class SkeletonAsset
		: public Asset							// 0x00
	{
	public:
		Array<String> m_boneNames;				// 0x0C
		Array<INT> m_hierarchy;					// 0x10
		Array<LinearTransform> m_localPose;		// 0x14
		Array<LinearTransform> m_modelPose;		// 0x18
		String m_weaponBoneName;				// 0x1C
		String m_headBoneName;					// 0x20
		String m_hipBoneName;					// 0x24
		String m_cameraBoneName;				// 0x28
	}; // 0x2C

	class PitchModifier
	{
	public:
		Vec3 m_offset;			// 0x00
		FLOAT m_pitchVal;		// 0x10
		FLOAT m_pitchAngle;		// 0x14
	}; // 0x18

	class BoneCollisionData
	{
	public:
		Vec3 m_debugDrawColor;							// 0x00
		Vec3 m_capsuleOffset;							// 0x10
		String m_boneName;								// 0x20
		HitReactionType m_animationHitReactionType;		// 0x24
		MaterialContainerPair* m_materialPair;			// 0x28
		INT m_boneAxis;									// 0x2C
		FLOAT m_capsuleLength;							// 0x30
		FLOAT m_capsuleRadius;							// 0x34
		PAD(0x8);										// 0x38
		PitchModifier m_minPitch;						// 0x40
		PAD(0x8);										// 0x58
		PitchModifier m_maxPitch;						// 0x60
		PAD(0x8);										// 0x78
		CHAR m_validInHiLod;							// 0x80
		CHAR m_validInLowLod;							// 0x81
		CHAR m_usePhysicsRotation;						// 0x82
		CHAR m_deactiveIfBehindWall;					// 0x83
		PAD(0xC);										// 0x84
	}; // 0x90

	class ClientBoneCollisionComponent
		: public ClientComponent,			// 0x00
		public BoneCollisionComponent		// 0x10
	{
	}; // 0x1E4

	class SoldierWeaponsComponent
	{
	public:
		LPVOID vftable;				// 0x00
		FLOAT m_weaponDispersion;	// 0x04
	}; // 0x08

	class WeaponSwitchingCallbacks
	{
		LPVOID vftable; // 0x00
	}; // 0x04

	class WeaponFiringCallbacks
	{
		LPVOID vftable; // 0x00
	}; // 0x04

	class WeaponSwitchingState
	{
	public:
		INT m_weaponCount;										// 0x00
		INT m_currentWeaponId;									// 0x04
		INT m_previousWeaponId;									// 0x08
		INT m_lastPrimary;										// 0x0C
		FLOAT m_weaponSwitchTimer;								// 0x10
		FLOAT m_switchBackToPrevMaxTimePressed;					// 0x14
		EntryInputActionEnum m_fireAndSwitchBackToPrevAction;	// 0x18
		CHAR m_automaticFire;									// 0x1C
		CHAR m_pPlayerSwitchedWeapons;							// 0x1D
		CHAR m_quickSwitch;										// 0x1E
		PAD(0x1);												// 0x1F
	}; // 0x20

	class WeaponsState
	{
	public:
		PAD(0xC4);		// 0x00
	}; // 0xC4

	class ClientWeaponsState
		: public WeaponsState	// 0x00
	{
	}; // 0xC4

	class AnimatedWeaponGS
	{
	public:
		class Read
		{
		public:
			FLOAT DisableZoomToggleWeight;		// 0x00
			INT AIAllowFire;					// 0x04
			INT AIAltFireFromAnt;				// 0x08
		}; // 0x0C

		class Write
		{
		public:
			CHAR Deploy;						// 0x00
			CHAR AltDeploy;						// 0x01
			CHAR Undeploy;						// 0x02
			CHAR QuickSwitch;					// 0x03
			CHAR Reload;						// 0x04
			CHAR ReloadShotgun;					// 0x05
			CHAR Fire;							// 0x06
			CHAR FireSingle;					// 0x07
			CHAR FireHoldAndRelease;			// 0x08
			CHAR FireSimple;					// 0x09
			CHAR FireShotSpawned;				// 0x0A
			CHAR BoltAction;					// 0x0B
			CHAR PumpAction;					// 0x0C
			CHAR MeleeAttack;					// 0x0D
			CHAR QuickThrow;					// 0x0E
			PAD(0x1);							// 0x0F
			INT QuickThrowType;					// 0x10
			CHAR AimBody;						// 0x14
			CHAR AlwaysAimHead;					// 0x15
			CHAR OneHanded;						// 0x16
			CHAR OneHandedAiming;				// 0x17
			CHAR AimingEnabled;					// 0x18
			CHAR LowerGun;						// 0x19
			CHAR BreathControl;					// 0x1A
			CHAR RflType;						// 0x1B
			CHAR PstlType;						// 0x1C
			CHAR HgrType;						// 0x1D
			CHAR ATType;						// 0x1E
			CHAR ShgType;						// 0x1F
			CHAR LMGType;						// 0x20
			CHAR BagType;						// 0x21
			CHAR SnpType;						// 0x22
			CHAR Zoom;							// 0x23
			FLOAT AimBodyWeight;				// 0x24
			FLOAT ZoomParameter;				// 0x28
			FLOAT ZoomScaleFactor;				// 0x2C
			FLOAT Dispersion;					// 0x30
			PAD(0xC);							// 0x34
			Vec3 AimTargetPosBody;				// 0x40
			FLOAT ZoomOutSpeed;					// 0x50
			FLOAT ZoomInSpeed;					// 0x54
			FLOAT UnDeploySpeed;				// 0x58
			FLOAT DeploySpeed;					// 0x5C
			CHAR LightEnabled;					// 0x60
			CHAR FireModeChanged;				// 0x61 Doesn't Update
			PAD(0x2);							// 0x62
			INT AnimType;						// 0x64
			INT GunDown;						// 0x68
			FLOAT NumberOfBulletsLeftInGun;		// 0x6C
			CHAR BulletsLeftInGun;				// 0x70
			CHAR WeaponActionESIG;				// 0x71
			CHAR IsSprinting;					// 0x72
			CHAR PreparingToBash;				// 0x73
			INT JustStartedSprinting;			// 0x74
			FLOAT KickBackInIronSight;			// 0x78
			FLOAT ZoomingTime;					// 0x7C
			INT TriggerZoomGunTwitch;			// 0x80
			INT WeaponChooserSignal;			// 0x84
			INT WeaponClassSignal;				// 0x88
			FLOAT OffsetX;						// 0x8C
			FLOAT OffsetY;						// 0x90
			FLOAT OffsetZ;						// 0x94
		}; // 0x98

		class Data
		{
		public:
			Read R;		// 0x00
			PAD(0x4);	// 0x0C
			Write W;	// 0x10
		}; // 0xA8

		Data m_data;		// 0x00
	}; // 0xA8
	class WeaponSway
	{
	public:
		class SoldierAttributesCallback
		{
		public:
			virtual CharacterPoseType getPose();									// V: 0x0
			virtual CharacterPoseType getChangingToPose();							// V: 0x4
			virtual D3DXVECTOR4 getSoldierSpeed();									// V: 0x8
			virtual FLOAT getSoldierMaxSpeed();										// V: 0xC
			virtual BOOL isWeaponZoomed();											// V: 0x10
			virtual SoldierEntityActionState actionState();							// V: 0x14
			virtual FLOAT getYawInputLevel();										// V: 0x18
			virtual FLOAT getPitchInputLevel();										// V: 0x1C
			virtual FLOAT getStrafeInputLevel();									// V: 0x20
			virtual FLOAT getThrottleInputLevel();									// V: 0x24
			virtual UINT getSeed();													// V: 0x28
			virtual BOOL isSupported();												// V: 0x2C
			virtual BOOL isWeaponLightEnabled();									// V: 0x30
			virtual void onDispersionUpdated();										// V: 0x34
			virtual BOOL isSprinting();												// V: 0x38
		}; // 0x04

		class AbilityCallback
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04

		class SuppressionCallback
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04
	};

	class WeaponSwayCallbackImpl
		: public WeaponSway::SoldierAttributesCallback					// 0x00
	{
	public:
		FLOAT m_yawInputLevel;											// 0x04
		FLOAT m_pitchInputLevel;										// 0x08
		FLOAT m_strafeInputLevel;										// 0x0C
		FLOAT m_throttleInputLevel;										// 0x10
		DWORD m_seed;													// 0x14
		PAD(0x8);														// 0x18
		Vec3 m_soldierSpeed;											// 0x20
		FLOAT m_soldierMaxSpeed;										// 0x30
		WeaponSway::AbilityCallback* m_abilityCallback;					// 0x34
		WeaponSway::SuppressionCallback* m_suppressionCallback;			// 0x38
		FLOAT m_currentSupressionLevel;									// 0x3C
		CharacterPoseType m_soldierPose;								// 0x40
		CharacterPoseType m_soldierChangingToPose;						// 0x44
		SoldierEntityActionState m_soldierActionState;					// 0x48
		CHAR m_isSupported;												// 0x4C
		CHAR m_isSprinting;												// 0x4D
		CHAR m_isWeaponLightEnabled;									// 0x4E
		CHAR m_isZooming;												// 0x4F
	}; // 0x50

	class ClientWeaponSwayCallbackImpl : public WeaponSwayCallbackImpl
	{
	public:
		int m_hasBeenLocalPlayer; //0x0050

	};//Size=0x0054


	class ClientSoldierWeaponsComponent
		: public ClientComponent,												// 0x00
		public network::IClientNetworkable,										// 0x10
		public network::Interpolator<LPVOID>,									// 0x18
		public SoldierWeaponsComponent,											// 0x58
		public WeaponSwitchingCallbacks,										// 0x60
		public WeaponFiringCallbacks											// 0x64
	{
	public:
		class ZoomCorrectionState
		{
			PAD(0x4);			// 0x00
			INT hasZoom;		// 0x04
			DWORD zoomLevel;	// 0x08
		}; // 0x0C

		class State
		{
		public:
			WeaponSwitchingState weaponSwitchingState;		// 0x00
			ZoomCorrectionState zoomCorrectionState;		// 0x20
		}; // 0x2C

		class ClientWeaponSwayCallbackImpl
			: public WeaponSwayCallbackImpl		// 0x00
		{
		public:
			INT m_hasBeenLocalPlayer;			// 0x50
		}; // 0x54

		DWORD m_correctionCallbackcHandler;										// 0x68
		PAD(0x4);																// 0x6C
		LinearTransform m_weaponTransform;										// 0x70
		DWORD m_animatableComponent[0x2];										// 0xB0
		ClientSoldierEntity* m_soldier;											// 0xB8
		eastl::vector<ClientSoldierWeapon2*> m_weapons;							// 0xBC
		ClientAnimatedSoldierWeaponHandler* m_currentAnimatedWeaponHandler;		// 0xCC
		INT m_weaponMeshWasEnabled;												// 0xD0
		WeaponSwitching* m_replicatedWeaponSwitching;							// 0xD4
		WeaponSwitching* m_predictedWeaponSwitching;							// 0xD8
		fb::ClientWeaponSwayCallbackImpl* m_weaponSwayCallback;						// 0xDC
		INT m_hasOrHadLocalPlayer;												// 0xE0
		SoldierAimingEnvironment* m_aimingEnvironment;							// 0xE4
		AimingConstraints* m_swimAimingConstraints;								// 0xE8
		PAD(0x64);																// 0xEC
		INT m_oldZoomState;														// 0x150
		INT m_lastZoomLevel;													// 0x154
		FLOAT m_timeSinceWeaponFire;											// 0x158
		CHAR m_needsWweapon1pResource;											// 0x15C
		CHAR m_weaponLightDisabled;												// 0x15D
		PAD(0x2);																// 0x15E
		State m_correctionState;												// 0x160
		State m_predictionState;												// 0x18C
		State* m_currentState;													// 0x1B8
		ClientWeaponsState m_currentWeaponsState;								// 0x1BC
		AnimatedWeaponGS m_gameState;											// 0x280
		PAD(0x28);																// 0x328
		LinearTransform m_globalAimOverride;									// 0x350
		INT m_globalAimOverrideMode;											// 0x390
		CHAR m_animationsDisabledForCurrentWeapon;								// 0x394
		CHAR m_instantReloadEnabled;											// 0x395
		CHAR m_flaggedForLowAmmo;												// 0x396
		PAD(0x1);																// 0x397
		AnimatedSoldierWeapon* m_lastAnimatedSoldierWeapon;						// 0x398
		DWORD m_1pWeaponAsset;													// 0x39C
		DWORD m_3pWeaponAsset;													// 0x3A0
	}; // 0x3A4

	class ClientAnimatedSoldierWeaponHandler
	{
	public:
		LPVOID vftable;									// 0x00
		CHAR m_weaponDeployRequested;					// 0x04
		CHAR m_altDeploy;								// 0x05
		CHAR m_quickSwitch;								// 0x06
		PAD(0x1);										// 0x07
		FLOAT m_weaponUndeployTimer;					// 0x08
		FLOAT m_spamDetectTimer;						// 0x0C
		PAD(0x4);										// 0x10
		ClientSoldierWeapon2* m_currentAnimatedWeapon;	// 0x14
		INT m_deleteCurrentAnimationWeapon;				// 0x18
		INT m_currentAnimatedWeaponIndex;				// 0x1C
		INT m_currentAnimatedWeaponState;				// 0x20
		ClientSoldierWeapon2* m_wantedAnimatedWeapon;	// 0x24
		INT m_wantedAnimatedWeaponIndex;				// 0x28
		ClientSoldierEntity* m_soldier;					// 0x2C
		INT m_lastPredicted;							// 0x30
	}; // 0x34

	class WeaponSwitching
	{
	public:
		LPVOID vftable;											// 0x00
		INT m_currentWeaponId;									// 0x04
		INT m_previousWeaponId;									// 0x08
		INT m_lastPrimary;										// 0x0C
		INT m_weaponCount;										// 0x10
		EntryInputActionEnum m_fireAndSwitchBackToPrevAction;	// 0x14
		FLOAT m_weaponSwitchTimer;								// 0x18
		FLOAT m_switchBackToPrevMaxTimePressed;					// 0x1C
		CHAR m_pPlayerSwitchedWeapons;							// 0x20
		CHAR m_quickSwitch;										// 0x21
		CHAR m_automaticFire;									// 0x22
		PAD(0x1);												// 0x23
		WeaponSwitchingCallbacks* m_callbackHandler;			// 0x24
	}; // 0x28

	class SoldierAimingEnvironment
	{
	public:
		ClientSoldierEntity* m_soldier;					// 0x00
		WeakPtr<ClientControllableEntity> m_target;		// 0x04
		CHAR m_hasTarget;								// 0x08
		CHAR m_hasFriendlyTarget;						// 0x09
		CHAR m_hasChangedTarget;						// 0x0A
		CHAR m_hasStickyBoxTarget;						// 0x0B
		CHAR m_hasSnapBoxTarget;						// 0x0C
		PAD(0x3);										// 0x0D
	}; // 0x10

	class AimingConstraints
	{
	public:
		FLOAT m_minYaw;				// 0x00
		FLOAT m_maxYaw;				// 0x04
		FLOAT m_minPitch;			// 0x08
		FLOAT m_maxPitch;			// 0x0C
		FLOAT m_pitchOffset;		// 0x10
		FLOAT m_yawOffset;			// 0x14
		FLOAT m_minYawDefault;		// 0x18
		FLOAT m_maxYawDefault;		// 0x20
	}; // 0x24

	class AnimatedSoldierWeaponSprintModule
	{
	public:
		CHAR m_wasSprinting;		// 0x00
		CHAR m_isSprinting;			// 0x01
		CHAR m_nearBashable;		// 0x02
		CHAR m_wasNearBashable;		// 0x03
	}; // 0x04

	class AnimatedSoldierWeaponShootModule
	{
	public:
		AnimationConfigurationShootModuleData* m_data;	// 0x00
		INT m_zooming;									// 0x04
	}; // 0x08

	class AnimationConfigurationShootModuleData
	{
	public:
		FLOAT m_zoomedKickbackFactor;		// 0x00
	}; // 0x04

	class AnimatedSoldierWeaponZoomModule
	{
	public:
		FLOAT m_zoomTimer;							// 0x00
		FLOAT m_oldZoomTimer;						// 0x04
		FLOAT m_isZooming;							// 0x08
		SoldierAimingSimulationData* m_aimingData;	// 0x0C
	}; // 0x10

	class AnimatedSoldierWeaponOffsetModule
	{
	public:
		WeaponOffsetData* m_data;		// 0x00
		INT m_zooming;					// 0x04
	}; // 0x08

	class WeaponOffsetData
		: public DataContainer			// 0x00
	{
	public:
		FLOAT m_weaponOffsetX;			// 0x08
		FLOAT m_weaponOffsetY;			// 0x0C
		FLOAT m_weaponOffsetZ;			// 0x10
		FLOAT m_weaponZoomedOffsetX;	// 0x14
		FLOAT m_weaponZoomedOffsetY;	// 0x18
		FLOAT m_weaponZoomedOffsetZ;	// 0x1C
	}; // 0x20

	class AnimatedSoldierWeaponSpeedModule
	{
	public:
		WeaponSpeedData* m_data;	// 0x00
	}; // 0x04

	class WeaponSpeedData
		: public DataContainer		// 0x00
	{
	public:
		FLOAT m_zoomOutSpeed;		// 0x08
		FLOAT m_zoomInSpeed;		// 0x0C
		FLOAT m_unDeploySpeed;		// 0x10
		FLOAT m_deploySpeed;		// 0x14
	}; // 0x18

	class AnimatedSoldierWeapon
		: WeaponFiringCallbacks								// 0x00
	{
	public:
		DWORD m_weapon;										// 0x04
		WeaponFiringData* m_weaponFiringData;				// 0x08
		WeaponModifier* m_weaponModifier;					// 0x0C
		DWORD m_weaponStateData;							// 0x10
		SoldierAimingSimulationData* m_aimingData;			// 0x14
		INT m_weaponType;									// 0x18
		AnimatedSoldierWeaponSprintModule m_sprintModule;	// 0x1C
		AnimatedSoldierWeaponShootModule m_shootModule;		// 0x20
		AnimatedSoldierWeaponZoomModule m_zoomModule;		// 0x28
		AnimatedSoldierWeaponOffsetModule m_offsetModule;	// 0x38
		AnimatedSoldierWeaponSpeedModule m_speedModule;		// 0x40
		PAD(0xC);											// 0x44
		Vec3 m_eyePosition;									// 0x50
		Vec3 m_aimAtPosition;								// 0x60
		FLOAT m_reloadTimer;								// 0x70
		FLOAT m_zoomTimer;									// 0x74
		FLOAT m_aimTimer;									// 0x78
		FLOAT m_aimBlendTimer;								// 0x7C
		FLOAT m_aimYaw;										// 0x80
		FLOAT m_aimPitch;									// 0x84
		FLOAT m_aimBodyBlendTime;							// 0x88
		FLOAT m_dispersion;									// 0x8C
		FLOAT m_disableZoomToggleWdight;					// 0x90
		FLOAT m_cameraLockWeight;							// 0x94
		INT m_bulletsLeftInGun;								// 0x98
		INT m_quickThrowType;								// 0x9C
		INT m_weaponAnimType;								// 0xA0
		INT m_animatedFireType;								// 0xA4
		INT m_fireLogicType;								// 0xA8
		CHAR m_isFiring;									// 0xAC
		PAD(0x3);
	}; // 0xD0

	class WeaponMiscModifierSettings
	{
	public:
		CHAR m_enableBreathControl;					// 0x00
		CHAR m_canBeInSupportedShooting;			// 0x01
		CHAR m_unZoomOnBoltAction;					// 0x02
		CHAR m_holdBoltActionUntilZoomRelease;		// 0x03
	}; // 0x04

	//class WeaponModifier
	//{
	//public:
	//char _0x0000[192];
	//	WeaponProjectileModifier* m_weaponProjectileModifier; //0x00C0
	//char _0x00A0[32];
	//};

	class WeaponModifier
	{
	public:
		PAD(0xA8);																	// 0x00
		WeaponMiscModifierSettings m_weaponMiscModifier;							// 0xA8
		PAD(0x4);																	// 0xAC
		WeaponFiringDataModifier* m_weaponFiringDataModifier;						// 0xB0
		WeaponFiringEffectsModifier* m_weaponFiringEffectsModifier;					// 0xB4
		WeaponSoundModifier* m_weaponSoundModifier;									// 0xB8
		WeaponShotModifier* m_weaponShotModifier;									// 0xBC
		WeaponProjectileModifier* m_weaponProjectileModifier;						// 0xC0
		WeaponAimingSimulationModifier* m_weaponAimingSimulationModifier;			// 0xC4
		WeaponAimingConfigurationModifier* m_weaponAimingConfigurationModifier;		// 0xC8
		WeaponAnimTypeModifier* m_weaponAnimTypeModifier;							// 0xCC
		WeaponMagazineModifier* m_weaponMagazineModifier;							// 0xD0
		WeaponZoomModifier* m_weaponZoomModifier;									// 0xD4
	}; // 0xD8

	class WeaponModifierBase
		: public DataContainer	// 0x00
	{
	}; // 0x08

	class WeaponFiringDataModifier
		: public WeaponModifierBase			// 0x00
	{
	public:
		DataContainer* m_weaponFiring;		// 0x08
	}; // 0x0C

	class FireEffectData
	{
	public:
		Vec3 m_rotation;				// 0x00
		Vec3 m_offset;					// 0x10
		Vec3 m_zoomRotation;			// 0x20
		Vec3 m_zoomOffset;				// 0x30
		DataContainer* m_effect;		// 0x40
		CHAR m_useZoomOffset;			// 0x44
		CHAR m_useZoomRotation;			// 0x45
		CHAR m_disableDuringZoom;		// 0x46
		CHAR m_updateTransform;			// 0x47
		CHAR m_stopLoopingEffects;		// 0x48
		PAD(0x3);						// 0x49
	}; // 0x4C

	class WeaponFiringEffectsModifier
		: public WeaponModifierBase				// 0x00
	{
	public:
		Array<FireEffectData> m_fireEffects1p;	// 0x08
		Array<FireEffectData> m_fireEffects3p;	// 0x0C
	}; // 0x10

	class WeaponSoundModifier
		: public WeaponModifierBase	// 0x00
	{
	public:
		DataContainer* m_sound;		// 0x08
	}; // 0x0C

	class WeaponShotModifier
		: public WeaponModifierBase			// 0x00
	{
	public:
		PAD(0x8);							// 0x08
		Vec3 m_initialSpeed;				// 0x10
		INT m_numberOfBulletsPerShell;		// 0x20
	}; // 0x24

	class WeaponProjectileModifier
		: public WeaponModifierBase				// 0x00
	{
	public:
		ProjectileEntityData* m_projectileData;	// 0x08
	}; // 0x0C

	class WeaponAimingSimulationModifier
		: public WeaponModifierBase				// 0x00
	{
	public:
		DataContainer* m_aimingController;		// 0x08
	}; // 0x0C

	class WeaponAimingConfigurationModifier
		: public WeaponModifierBase						// 0x00
	{
	public:
		Array<FLOAT> m_zoomInOutMeshTransitionFactors;	// 0x08
		FLOAT m_zoomedKickBackFactor;					// 0x0C
		DataContainer* m_weaponOffsetModuleData;		// 0x10
		DataContainer* m_weaponSpeedModuleData;			// 0x14
	}; // 0x18

	class WeaponAnimTypeModifier
		: public WeaponModifierBase			// 0x00
	{
	public:
		WeaponAnimType m_weaponAnimType;	// 0x08
	}; // 0x0C

	class WeaponMagazineModifier
		: public WeaponModifierBase		// 0x00
	{
	public:
		INT m_magazineCapacity;			// 0x08
		INT m_numberOfMagazines;		// 0x0C
	}; // 0x10

	class WeaponZoomModifier
		: public WeaponModifierBase		// 0x00
	{
	public:
		FLOAT m_zoomRenderFov;			// 0x08
	}; // 0x0C

	class ClientSoldierWeapon_old
		: public ClientGameEntity								// 0x00
	{
	public:
		class WeaponState
		{
		public:
			AnimatedSoldierWeapon* m_animation;				// 0x00
			eastl::vector<DWORD> m_fakePhysics;				// 0x04 <BoneFakePhysics*>
			DWORD m_meshVariationSet;						// 0x14 MeshVariationSet*
			DWORD m_meshZoomVariationSet;					// 0x18 MeshVariationSet*
			DWORD m_mesh;									// 0x1C ResourceProxy<MeshSet>
			DWORD m_meshZoom;								// 0x20 ResourceProxy<MeshSet>
			WORD m_meshInstanceHandle;						// 0x24
			WORD m_meshZoomInstanceHandle;					// 0x28
			DWORD m_projectileBoneIndex;					// 0x2C
		}; // 0x30

		class CorrectionCallbackHandler
			: public WeaponFiringCallbacks	// 0x00
		{
		public:
			AmmunitionDepot* m_ammoDepot;	// 0x04
		}; // 0x08

		PAD(0x8);												// 0x20
		eastl::vector<WeaponState> m_weaponStates1p;			// 0x28
		eastl::vector<WeaponState> m_weaponStates3p;			// 0x38
		eastl::vector<DWORD> m_socketPointers;					// 0x48 AttachableSocket (Size 0xA4)
		WeaponModifier m_weaponModifier;						// 0x58
		PAD(0x10);												// 0x130
		DWORD m_cameraLagEffect;								// 0x13C
		ClientSoldierAimingSimulation* m_authorativeAiming;		// 0x140 WeaponLagEffect*
		ClientAimingReplication* m_replicatedAiming;			// 0x144
		ClientLockingController* m_lockingController;			// 0x148
		ClientLockingController* m_secondaryLockingController;	// 0x14C
		ClientWeapon* m_weapon;									// 0x150
		ClientWeaponFiringReplication* m_replicatedFiring;		// 0x154
		CorrectionCallbackHandler* m_correctionCallbackHandler;	// 0x158
		WeaponFiring* m_predictedFiring;						// 0x15C
		WeaponFiring* m_correctedFiring;						// 0x160
		ResourceCompartment m_compartment;						// 0x164
		WORD m_weaponInstanceId;								// 0x168
		PAD(0x2);												// 0x16A
		INT m_projectileBoneIndex;								// 0x16C
		DWORD m_1pResourceCompartment;							// 0x170 SoldierWeaponResourceCompartment*
	};

	class ClientSoldierWeapon
		: public ClientGameEntity								// 0x00
	{
	public:
		class WeaponState
		{
		public:
			AnimatedSoldierWeapon* m_animation;				// 0x00
			eastl::vector<DWORD> m_fakePhysics;				// 0x04 <BoneFakePhysics*>
			DWORD m_meshVariationSet;						// 0x14 MeshVariationSet*
			DWORD m_meshZoomVariationSet;					// 0x18 MeshVariationSet*
			DWORD m_mesh;									// 0x1C ResourceProxy<MeshSet>
			DWORD m_meshZoom;								// 0x20 ResourceProxy<MeshSet>
			WORD m_meshInstanceHandle;						// 0x24
			WORD m_meshZoomInstanceHandle;					// 0x28
			DWORD m_projectileBoneIndex;					// 0x2C
		}; // 0x30

		class CorrectionCallbackHandler
			: public WeaponFiringCallbacks	// 0x00
		{
		public:
			AmmunitionDepot* m_ammoDepot;	// 0x04
		}; // 0x08

		BYTE pad_010[0x24]; // 0x10
		DWORD m_ownedEntities;
		INT m_ownedEntityCount;
		eastl::vector<DWORD> m_weaponStates1p;
		eastl::vector<DWORD> m_weaponStates3p;
		eastl::vector<DWORD> m_socketPoints;
		WeaponModifier m_weaponModifier;
		BYTE pad_130[0xC];
		DWORD m_cameraLagEffect;  //WeaponLagEffect*
		ClientSoldierAimingSimulation* m_authorativeAiming;
		ClientAimingReplication* m_replicatedAiming;
		DWORD m_lockingController;
		DWORD m_secondaryLockingController;
		ClientWeapon* m_weapon;
		DWORD m_replicatedFiring;
		DWORD m_correctionCallbackHandler;
		WeaponFiring* m_predictedFiring;
		WeaponFiring* m_correctedFiring;
		INT m_compartment;
		WORD m_weaponInstanceId;
		BYTE pad_16A[0x2];
		INT m_projectileBoneIndex;
		DWORD m_lpResourceCompartment;
		BYTE m_hasAttachedEntities;
		BYTE m_willAlwaysBe3p;
	};


	class ClientSoldierWeapon2
	{
	public:
		BYTE pad_010[0x20]; // 0x00
		DWORD m_ownedEntities;  //0x20
		INT m_ownedEntityCount;  //0x24
		eastl::vector<DWORD> m_weaponStates1p; //0x28
		eastl::vector<DWORD> m_weaponStates3p; //0x38
		eastl::vector<DWORD> m_socketPoints; //0x48
		WeaponModifier m_weaponModifier; //0x58
		BYTE pad_130[0x10]; //0x130
		ClientSoldierAimingSimulation* m_authorativeAiming; //0x140
		ClientSoldierAimingSimulation* m_authorativeAiming2; //0x144
		ClientAimingReplication* m_replicatedAiming;
		DWORD m_lockingController;
		DWORD m_secondaryLockingController;
		ClientWeapon* m_weapon;
		ClientWeaponFiringReplication* m_replicatedFiring;
		DWORD m_correctionCallbackHandler;
		WeaponFiring* m_predictedFiring;
		WeaponFiring* m_correctedFiring;
		INT m_compartment;
		WORD m_weaponInstanceId;
		BYTE pad_16A[0x2];
		INT m_projectileBoneIndex;
		DWORD m_lpResourceCompartment;
		BYTE m_hasAttachedEntities;
		BYTE m_willAlwaysBe3p;
	};


	class AmmunitionDepot
	{
	public:
		LPVOID vftable;		// 0x00
	}; // 0x04

	class AimAssist2
	{
	public:
		char _0x0000[12];
		float m_yaw; //0x00C
		UCHAR unknown_10[8];
		float m_pitch; //0x018
		char _0x0014[32];
		bool m_useAimAssist; //0x038

	};//Size=0x0058
	class AimAssist
	{
	public:
		class UpdateContext
		{
		public:
			FLOAT deltaTime;				// 0x00
			Vec2 movementInput;				// 0x04
			Vec2 aimingInput;				// 0x0C
			FLOAT minimumPitch;				// 0x14
			FLOAT maximumPitch;				// 0x18
			FLOAT lookSpeedmUltiplier;		// 0x1C
			FLOAT aimScale;					// 0x20
			PAD(0x10);					// 0x24
			LinearTransform reticuleSpace;	// 0x30
			Vec3 reticuleSpeed;				// 0x70
			FLOAT reticuleFieldOfView;		// 0x80
			CHAR hasStickyTarget;			// 0x84
			CHAR hasSnapTarget;				// 0x85
			CHAR forceAimSnap;				// 0x86
			CHAR usePitchZoomSnap;			// 0x87
			FLOAT attractZoomMultiplier;	// 0x88
			FLOAT targetRadius;				// 0x8C
			Vec3 targetPosition;			// 0x90
			Vec3 targetSpeed;				// 0xA0
			DWORD zoomLevel;				// 0xB0
			FLOAT zoomTransitionTimer;		// 0xB4
			INT isMouseInput;				// 0xB8
		}; // 0xBC

		SoldierAimAssistData* m_data;				// 0x00
		AimingConstraints* m_aimingConstraints;		// 0x04
		DWORD m_lastZoomLevel;						// 0x08
		FLOAT m_yaw;
		PAD(0x08);// áàóíòè
		FLOAT m_pitch;								// 0x10
		Vec2 m_speed;								// 0x14
		FLOAT m_acceleration;						// 0x1C
		FLOAT m_accelerationTimer;					// 0x20
		FLOAT m_attractZoomPostTimer;				// 0x24
		FLOAT m_targetDistance;						// 0x28
		FLOAT m_softZoneLambda;						// 0x2C
		FLOAT m_polynomialFactor;					// 0x30
		CHAR m_useAimAssist;						// 0x34
		CHAR m_useInputPolynomials;					// 0x35
		CHAR m_hasChangedZoomed;					// 0x36
		CHAR m_isSnapZoom;							// 0x37
	}; // 0x38
	class ClientSoldierAimingSimulation
	{
	public:
		enum ZoomFadeState
		{
			NoFade,
			Fading,
		};

		SoldierAimingSimulationData* m_data;					// 0x00
		SoldierAimingEnvironment* m_environment;				// 0x04
		AimAssist* m_fpsAimer;									// 0x08
		FLOAT m_yaw;											// 0x0C
		FLOAT m_pitch;											// 0x10
		FLOAT m_aimYawTimer;									// 0x14
		FLOAT m_aimPitchTimer;									// 0x18
		Vec2 m_sway;											// 0x1C
		PAD(0xC);												// 0x24
		AimAssist::UpdateContext m_updateContext;				// 0x30
		PAD(0x4);												// 0xEC
		DWORD m_zoomLevel;										// 0xF0
		DWORD m_oldZoomLevel;									// 0xF4
		DWORD m_switchToZoomLevel;								// 0xF8
		FLOAT m_zoomTransitionTimer;							// 0xFC
		FLOAT m_fovMultiplier;									// 0x100
		FLOAT m_fovTransitionTimer;								// 0x104
		PAD(0x4);												// 0x108
		ZoomFadeState m_fadeState;								// 0x10C
		FLOAT m_fadeTimer;										// 0x110
		eastl::vector<AimerModifierData*> m_modifierData;		// 0x114
		FLOAT m_currentZoomTransitionTime;						// 0x124
		FLOAT m_currentFovTransitionTime;						// 0x128
	}; // 0x12C

	class ZoomLevelData
		: public DataContainer
	{
	public:
		FLOAT m_fieldOfView;								// 0x08
		FLOAT m_timeYawMultiplier;							// 0x0C
		FLOAT m_lookSpeedMultiplier;						// 0x10
		FLOAT m_sprintLookSpeedMultiplier;					// 0x14
		FLOAT m_moveSpeedMultiplier;						// 0x18
		FLOAT m_swayPitchMultiplier;						// 0x1C
		FLOAT m_swayYawMultiplier;							// 0x20
		FLOAT m_supportedSwayPitchMultiplier;				// 0x24
		FLOAT m_supportedSwayYawMultiplier;					// 0x28
		FLOAT m_timePitchMultiplier;						// 0x2C
		FLOAT m_dispersionMultiplier;						// 0x30
		FLOAT m_startFadeToBlackAtTime;						// 0x34
		FLOAT m_recoilMultiplier;							// 0x38
		FLOAT m_recoilFovMultiplier;						// 0x3C
		FLOAT m_cameraImpulseMultiplier;					// 0x40
		ZoomLevelActivateEventType m_onActivateEventType;	// 0x44
		FLOAT m_startFadeFromBlackAtTime;					// 0x48
		FLOAT m_fadeToBlackDuration;						// 0x4C
		FLOAT m_fadeFromBlackDuration;						// 0x50
		FLOAT m_unknown;									// 0x54
		CHAR m_fadeToBlackInZoomTransition;					// 0x58
		CHAR m_useFovSpecialisation;						// 0x59
		CHAR m_allowFieldOfViewScaling;						// 0x5A
		PAD(0x1);											// 0x5B
	}; // 0x5C

	class AimingPoseData
	{
	public:
		FLOAT m_minimumPitch;		// 0x00
		FLOAT m_maximumPitch;		// 0x04
		FLOAT m_targetingFov;		// 0x08
		FLOAT m_aimSteadiness;		// 0x0C
		FLOAT m_speedMultiplier;	// 0x10
		FLOAT m_recoilMultiplier;	// 0x14
	}; // 0x18

	class AimerModifierData
		: public Asset						// 0x00
	{
	public:
		FLOAT m_lookSpeedMultiplier;		// 0x0C
		INT m_onlyInSupportedShooting;		// 0x10
	}; // 0x14

	class SoldierAimingSimulationData
		: public GameDataContainer					// 0x00
	{
	public:
		RefArray<ZoomLevelData> m_zoomLevels;		// 0x08
		FLOAT m_zoomTransitionTime;					// 0x0C
		DataContainer* m_aimAssist;					// 0x10
		AimingPoseData m_standPose;					// 0x14
		AimingPoseData m_crouchPose;				// 0x2C
		AimingPoseData m_pronePose;					// 0x44
		DWORD m_zoomTransitionTimeArray;			// 0x5C
		FLOAT m_fovTransitionTime;					// 0x60
		FLOAT m_fovDelayTime;						// 0x64
		Array<AimerModifierData*> m_modifiers;		// 0x68
		FLOAT m_aimingRange;						// 0x6C
		FLOAT m_lockedAimToTargetSpeed;				// 0x70
		INT m_rootToZoomAfterReload;				// 0x74
	}; // 0x78

	class SoldierAimAssistData
		: public GameDataContainer								// 0x00
	{
	public:
		PAD(0x8);												// 0x08
		Vec3 m_eyePosOffset;									// 0x10
		Vec3 m_stickyBoxScale;									// 0x20
		Vec3 m_snapDistanceScale;								// 0x30
		Vec3 m_snapBoxScale;									// 0x40
		Vec3 m_stickyDistanceScale;								// 0x50
		Vec3 m_maxAcceleration;									// 0x60
		FLOAT m_accelerationDamping;							// 0x68
		FLOAT m_accelerationInputThreshold;						// 0x6C
		FLOAT m_accelerationMultplier;							// 0x70
		FLOAT m_squaredAcceleration;							// 0x74
		FLOAT m_yawSpeedStrength;								// 0x78
		Array<FLOAT> m_zoomedInputPolynomial;					// 0x7C
		FLOAT m_accelerationTimeThreshold;						// 0x80
		Array<FLOAT> m_attractDistanceFallOff;					// 0x84
		FLOAT m_attractUserInputMultiplier;						// 0x88
		FLOAT m_attractOwnSpeedInfluence;						// 0x8C
		FLOAT m_attractTargetSpeedInfluence;					// 0x90
		FLOAT m_attractOwnRequiredMovementForMaximumAttract;	// 0x94
		FLOAT m_attractStartInputThreshold;						// 0x98
		FLOAT m_attractZoomingMultiplier;						// 0x9C
		FLOAT m_attractZoomingPostTime;							// 0xA0
		FLOAT m_attractYawStrength;								// 0xA4
		FLOAT m_attractPitchStrength;							// 0xA8
		FLOAT m_pitchSpeedStrength;								// 0xAC
		FLOAT m_attractSoftZone;								// 0xB0
		Array<FLOAT> m_inputPolynomial;							// 0xB4
		CHAR m_useYawAcceleration;								// 0xB8
		CHAR m_usePitchAcceleration;							// 0xB9
		PAD(0x2);												// 0xBA
	}; // 0xBC

	class ClientAimingReplication
	{
	public:
		LPVOID vftable;			// 0x00
		FLOAT m_yaw;			// 0x04
		FLOAT m_pitch;			// 0x08
		FLOAT m_deltaYaw;		// 0x0C
		FLOAT m_deltaPitch;		// 0x10
		FLOAT m_activated;		// 0x14
	}; // 0x18

	class LockingController
	{
	public:
		enum LockState
		{
			Idle,
			Locking,
			Locked,
		};

		LPVOID vftable;					// 0x00
		LockingControllerData* m_data;	// 0x04
		FLOAT m_currLockAmount;			// 0x08
		LockState m_lockState;			// 0x0C
		CHAR m_active;					// 0x10
		CHAR m_lastLockSuccessful;		// 0x11
		PAD(0x2);						// 0x12
		DWORD m_zoomLevel;				// 0x14
		LockType m_currentLockType;		// 0x18
	}; // 0x1C

	class ZoomLevelLockData
	{
	public:
		FLOAT m_outlineTaggedDistance;		// 0x00
		LockType m_lockType;				// 0x04
	}; // 0x08

	class LockingControllerData
		: public DataContainer
	{
	public:
		Array<ZoomLevelLockData> m_zoomLevelLock;	// 0x08
		FLOAT m_lockTime; 							// 0x0C
		FLOAT m_releaseTime; 						// 0x10
		FLOAT m_releaseOnNewTargetTime;				// 0x14
		FLOAT m_sampleRate; 						// 0x18
		FLOAT m_holdStillThreshold; 				// 0x1C
		FLOAT m_rayLength; 							// 0x20
		FLOAT m_minimumLockTime; 					// 0x24
		FLOAT m_acceptanceAngle; 					// 0x28
		FLOAT m_angleConstant; 						// 0x2C
		FLOAT m_sensitivity; 						// 0x30
		FLOAT m_distanceConstant; 					// 0x34
		CHAR m_positionOnly; 						// 0x38
		CHAR m_lockOnWorldSpacePos; 				// 0x39
		CHAR m_lockOnVisibleTargetsOnly;			// 0x3A
		CHAR m_lockOnEmptyVehicles; 				// 0x3B
	};

	class ClientLockingController
		: public LockingController					// 0x00
	{
	public:
		WeakPtr<ClientPhysicsEntity> m_currTarget;	// 0x1C
		Vec3 m_targetPos;							// 0x20
		WORD m_targetGhostId;						// 0x30
		PAD(0x2);									// 0x32
	}; // 0x34

	class WeaponFiringCallbackHandler
	{
	public:
		class Event
		{
		public:
			WeaponFiringEvent call;	// 0x00
			FLOAT powerModifier;	// 0x04
			DWORD ticks;			// 0x08
			CHAR trace;				// 0x0C
			CHAR detonatorActive;	// 0x0D
			CHAR needUpdate;		// 0x0E
			PAD(0x1);				// 0x0F
		}; // 0x10

		LPVOID vftable;		// 0x00
		PAD(0x88);			// 0x04
	}; // 0x8C

	class ClientWeaponFiringReplication
		: public WeaponFiringCallbackHandler	// 0x00
	{
	public:
		class AIWeaponData
		{
		public:
			INT fireCount;					// 0x00
			INT releaseCount;				// 0x04
			INT autoFireCount;				// 0x08
			FLOAT timeSinceFiring;			// 0x0C
			FLOAT powerModifier;			// 0x10
			eastl::vector<UINT> tickBuffer;	// 0x14
			CHAR wasAutomaticing;			// 0x24
			CHAR isHolding;					// 0x25
			CHAR hasSpawnedShot;			// 0x26
			PAD(0x1);						// 0x27
		}; // 0x28

		WeaponFiringData* m_data;				// 0x8C
		WeaponModifier* m_weaponModifier;		// 0x90
		CHAR m_networkFlagIsSingleFiring;		// 0x94
		CHAR m_networkFlagIsAutomaticFiring;	// 0x95
		CHAR m_networkFlagIsReloading;			// 0x96
		CHAR m_networkFlagBoltAction;			// 0x97
		AIWeaponData m_primary;					// 0x98
		FLOAT m_timeUntilNextAutomaticShot;		// 0xC0
		INT m_reloadCount;						// 0xC4
		INT m_boltActionCount;					// 0xC8
		DWORD m_firedBullets;					// 0xCC
		FLOAT m_heat;							// 0xD0
		FLOAT m_timeToWait;						// 0xD4
		eastl::vector<Event> m_events;			// 0xD8
		WeaponSway* m_weaponSway;				// 0xE8
		bool m_activated;						// 0xEC
		bool m_detonatorActive;					// 0xED
		bool m_oldDetonatorState;				// 0xEE
		bool m_firedCount;						// 0xEF
		bool m_firstApply;						// 0xF0
		bool m_useAIShootSpace;					// 0xF1
		PAD(0x3E);								// 0xF2
		LinearTransform m_aiShootSpace;			// 0x130
		INT m_fireLogicType;					// 0x170
		FLOAT m_simulatedTime;					// 0x174
		FLOAT m_predictedTime;					// 0x178
	}; // 0x17C

	class WeaponFiringData
		: public GameDataContainer				// 0x00
	{
	public:
		fb::CtrRefBase< FiringFunctionData > m_primaryFire;
		//FiringFunctionData* m_primaryFire;		// 0x08
		FLOAT m_deployTime;						// 0x0C
		FLOAT m_reactivateCooldownTime;			// 0x10
		FLOAT m_altDeployTime;					// 0x14
		PAD(0x4);								// 0x18
		INT m_shotLimit;						// 0x1C
		DataContainer* m_weaponSway;			// 0x20
		FLOAT m_supportDelayProne;				// 0x24
	}; // 0x28

	class FiringDispersionData
	{
	public:
		FLOAT m_minAngle;			// 0x00
		FLOAT m_maxAngle;			// 0x04
		FLOAT m_increasePerShot;	// 0x08
		FLOAT m_decreasePerShot;	// 0x0C
	}; // 0x10

	class SoldierWeaponDispersion
	{
	public:
		FiringDispersionData m_standDispersion;		// 0x00
		FiringDispersionData m_crouchDispersion;	// 0x10
		FiringDispersionData m_proneDispersion;		// 0x20
		FLOAT m_jumpDispersionAngle;				// 0x30
		FLOAT m_proneTransitionDispersionAngle;		// 0x34
		FLOAT m_moveDispersionAngle;				// 0x38
		FLOAT m_moveZoomedDispersionAngle;			// 0x3C
		FLOAT m_decreasePerSecond;					// 0x40
	}; // 0x44

	class ShotConfigData
	{
	public:
		Vec3 m_initialPosition; 							// 0x00
		Vec3 m_initialDirection; 							// 0x10
		Vec3 m_initialSpeed; 								// 0x20 //.z = Bulletspeed
		FLOAT m_inheritWeaponSpeedAmount; 					// 0x30
		DWORD m_muzzleExplosion; 							// 0x34
		ProjectileEntityData* m_projectileData; 			// 0x38
		ProjectileEntityData* m_secondaryProjectileData; 	// 0x3C
		DWORD m_projectile; 								// 0x40
		DWORD m_secondaryProjectile; 						// 0x44
		FLOAT m_spawnDelay; 								// 0x48
		DWORD m_numberOfBulletsPerShell;				 	// 0x4C
		DWORD m_numberOfBulletsPerShot;						// 0x50
		DWORD m_numberOfBulletsPerBurst;					// 0x54
		CHAR m_relativeTargetAiming; 						// 0x58
		CHAR m_forceSpawnToCamera; 							// 0x59
		CHAR m_spawnVisualAtWeaponBone;						// 0x5A
		CHAR m_activeForceSpawnToCamera;				 	// 0x5B
	}; // 0x5C

	class ProjectileEntityData
		: public GamePhysicsEntityData				// 0x00
	{
	public:
		PAD(0xC);									// 0x64
		INT m_hitReactionWeaponType;				// 0x70
		FLOAT m_initialSpeed;						// 0x74
		FLOAT m_timeToLive;							// 0x78
		FLOAT m_initMeshHideTime;					// 0x7C
		FLOAT m_visualConvergeDistance;				// 0x80
		FLOAT m_unknown;							// 0x84
		MaterialContainerPair* m_materialPair;		// 0x88
		DataContainer* m_explosion;					// 0x8C
		WeaponSuppressionData* m_suppressionData;	// 0x90
		String m_ammunitionType;					// 0x94
		CHAR m_serverProjectileDisabled;			// 0x98
		CHAR m_detonateOnTimeout;					// 0x99
		PAD(0x26);									// 0x9A
		float m_stamina;                     // this+0xC0
		PAD(0x08);
		float m_gravity;                     // this+0xCC
		float m_impactImpulse;                     // this+0xD0
		float m_detonationTimeVariation;                     // this+0xD4
		float m_vehicleDetonationRadius;                     // this+0xD8
		float m_vehicleDetonationActivationDelay;                     // this+0xDC
		float m_flyBySoundRadius;                     // this+0xE0
		float m_flyBySoundSpeed;                     // this+0xE4
		float m_firstFrameTravelDistance;                     // this+0xE8
		float m_distributeDamageOverTime;                     // this+0xEC
		float m_startDamage;                     // this+0xF0
		float m_endDamage;                     // this+0xF4
		float m_damageFalloffStartDistance;                     // this+0xF8
		float m_damageFalloffEndDistance;                     // this+0xFC
		float m_timeToArmExplosion;                     // this+0x100
		bool m_hasVehicleDetonation;                     // this+0x104
		bool m_instantHit;                     // this+0x105
		bool m_stopTrailEffectOnUnspawn;                     // this+0x106

	}; // 0x9C

	class WeaponSuppressionData
		: public DataContainer		// 0x00
	{
	public:
		FLOAT m_maxMultiplier;		// 0x08
		FLOAT m_minMultiplier;		// 0x0C
		FLOAT m_minDistance;		// 0x10
		FLOAT m_maxDistance;		// 0x14
	}; // 0x18

	class HoldAndReleaseData
	{
	public:
		FLOAT m_maxHoldTime;					// 0x00
		FLOAT m_minPowerModifier;				// 0x04
		FLOAT m_maxPowerModifier;				// 0x08
		FLOAT m_powerIncreasePerSecond;			// 0x0C
		FLOAT m_delay;							// 0x10
		FLOAT m_killedHoldingPowerModifier;		// 0x14
		INT m_forceFireWhenKilledHolding;		// 0x18
	}; // 0x1C

	class BoltActionData
	{
	public:
		FLOAT m_boltActionDelay;				// 0x00
		FLOAT m_boltActionTime;					// 0x04
		CHAR m_holdBoltActionUntilFireRelease;	// 0x08
		CHAR m_holdBoltActionUntilZoomRelease;	// 0x09
		CHAR m_forceBoltActionOnFireTrigger;	// 0x0A
		CHAR m_unZoomOnBoltAction;				// 0x0B
		CHAR m_returnToZoomAfterBoltAction;		// 0x0C
		PAD(3);									// 0x0D
	}; // 0x10

	class RecoilData
	{
	public:
		FLOAT m_maxRecoilAngleX;			// 0x00
		FLOAT m_minRecoilAngleX;			// 0x04
		FLOAT m_maxRecoilAngleY;			// 0x08
		FLOAT m_minRecoilAngleY;			// 0x0C
		FLOAT m_maxRecoilAngleZ;			// 0x10
		FLOAT m_minRecoilAngleZ;			// 0x14
		FLOAT m_maxRecoilFov;				// 0x18
		FLOAT m_minRecoilFov;				// 0x1C
		INT m_recoilFollowsDispersion;		// 0x20
	}; // 0x24
	class FireLogicData
	{
	public:
		HoldAndReleaseData m_holdAndRelease;				// 0x00
		BoltActionData m_boltAction;						// 0x1C
		RecoilData m_recoil;								// 0x2C
		EntryInputActionEnum m_fireInputAction; 			// 0x50
		EntryInputActionEnum m_reloadInputAction; 			// 0x54
		EntryInputActionEnum m_cycleFireModeInputAction; 	// 0x58
		FLOAT m_triggerPullWeight; 							// 0x5C
		FLOAT m_rateOfFire; 								// 0x60
		FLOAT m_rateOfFireForBurst; 						// 0x64
		FLOAT m_clientFireRateMultiplier; 					// 0x68
		FLOAT m_reloadDelay; 								// 0x6C
		Array<FireLogicType> m_fireLogicTypeArray; 			// 0x70
		FLOAT m_reloadThreshold; 							// 0x74
		FLOAT m_preFireDelay; 								// 0x78
		FLOAT m_reloadTime; 								// 0x7C
		FLOAT m_reloadTimeBulletsLeft; 						// 0x80
		FireLogicType m_fireLogicType; 						// 0x84
		ReloadLogic m_reloadLogic; 							// 0x88
		FLOAT m_automaticDelay; 							// 0x8C
		ReloadType m_reloadType; 							// 0x90
		CHAR m_holdOffReloadUntilZoomRelease; 				// 0x94
		CHAR m_forceReloadActionOnFireTrigger; 				// 0x95
		CHAR m_holdOffReloadUntilFireRelease; 				// 0x96
		CHAR m_alwaysAutoReload; 							// 0x97
	}; // 0x98
	class AmmoConfigData
	{
	public:
		INT m_magazineCapacity; 				// 0x00
		INT m_numberOfMagazines; 				// 0x04
		DWORD m_traceFrequency; 				// 0x08
		DWORD m_ammoPickupMinAmount; 			// 0x0C
		DWORD m_ammoPickupMaxAmount; 			// 0x10
		FLOAT m_autoReplenishDelay; 			// 0x14
		INT m_ammoBagPickupAmount; 				// 0x18
		FLOAT m_ammoBagPickupDelayMultiplier; 	// 0x1C
		INT m_autoReplenishMagazine; 			// 0x20
	}; // 0x24
	class OverHeatData
	{
	public:
		FLOAT m_heatPerBullet;				// 0x00
		FLOAT m_heatDropPerSecond;			// 0x04
		FLOAT m_overHeatPenaltyTime;		// 0x08
		FLOAT m_overHeatThreshold;			// 0x0C
		FireEffectData m_overHeatEffect;	// 0x10
	}; // 0x5C
	class FiringFunctionData
		: public DataContainer						// 0x00
	{
	public:
		Array<FiringDispersionData> m_dispersion;	// 0x08
		SoldierWeaponDispersion m_weaponDispersion;	// 0x0C
		Array<FireEffectData> m_fireEffects1p;		// 0x50
		Array<FireEffectData> m_fireEffects3p;		// 0x54
		DataContainer* m_sound;						// 0x58
		PAD(0x4);									// 0x5C
		ShotConfigData m_shot;						// 0x60
		PAD(0x4);									// 0xBC
		FireLogicData m_fireLogic;					// 0xC0
		AmmoConfigData m_ammo;						// 0x158
		PAD(0x4);									// 0x17C
		OverHeatData m_overHeat;					// 0x180
		PAD(0x4);									// 0x1DC
		FLOAT m_selfHealTimeWhenDeployed;			// 0x1E0
		FLOAT m_ammoCrateReloadDelay;				// 0x1E4
		CHAR m_unlimitedAmmoForAI;					// 0x1E8
		CHAR m_usePrimaryAmmo;						// 0x1E9
		PAD(0x2);									// 0x1EA
	}; // 0x1EC
	class RefillableAmmunitionDepot
		: public AmmunitionDepot
	{
	};
	class WeaponFiring
		: public WeaponFiringCallbackHandler,			// 0x00
		public RefillableAmmunitionDepot				// 0x8C
	{
	public:
		enum WeaponState
		{
			Deploy,
			AltDeploy,
			TriggerReleaseWait,
			NoTrigger,
			PreFireDelay,
			PrimarySingleDelay,
			PrimarySingle,
			BoltActionDelay,
			BoltAction,
			PrimaryAutomaticFire,
			ReloadDelay,
			Reload,
			PostReload,
			PrimaryHoldAndRelease_Hold,
			PrimaryHoldAndRelease_ReleaseDelay,
			PrimaryHoldAndRelease_Release,
			NumWeaponStates,
			WeaponStateSizeInBits,
		};

		class Function
		{
		public:
			INT projectilesLoaded; 						// 0x00
			INT projectilesInMagazines; 				// 0x04
			INT numberOfProjectilesToFire; 				// 0x08
			CHAR hasStoppedFiring; 						// 0x0C
			CHAR primaryFireTriggeredLastFrame; 		// 0x0D
			CHAR isOverheated; 							// 0x0E
			PAD(0x1);									// 0x0F
			FLOAT heat; 								// 0x10
			FLOAT overheatTimer; 						// 0x14
			DWORD ticks; 								// 0x18
			DWORD currentFireModeIndex; 				// 0x1C
			INT externalMagazineCapacity; 				// 0x20
			GameObjectData* m_firingHolderData; 		// 0x24
			eastl::vector<FireLogicType> m_fireModes; 	// 0x28
		};

		MemoryArena* m_arena;							// 0x90
		WeaponFiringData* m_data;						// 0x94
		AmmoConfigData* m_ammoData;						// 0x98
		GameObjectData* m_firingHolderData;				// 0x9C
		//fb::ScopedPtr<WeaponSway>m_weaponSway;
		WeaponSway* m_weaponSway;						// 0xA0
		WeaponState m_weaponState;						// 0xA4
		WeaponState m_lastWeaponState;					// 0xA8
		WeaponState m_nextWeaponState;					// 0xAC
		FLOAT m_switchCooldownTimer;					// 0xB0
		FLOAT m_autoReplenishTime;						// 0xB4
		FLOAT m_timeToWait;								// 0xB8
		FLOAT m_stateData;								// 0xBC
		FLOAT m_holdReleaseMinDelay;					// 0xC0
		FLOAT m_recoilTimer;							// 0xC4
		FLOAT m_recoilAngleX;							// 0xC8
		FLOAT m_recoilAngleY;							// 0xCC
		FLOAT m_recoilAngleZ;							// 0xD0
		FLOAT m_recoilFovAngle;							// 0xD4
		FLOAT m_primaryAmmoReplenishDelay;				// 0xD8
		FLOAT m_reloadTimeMultiplier;					// 0xDC
		FLOAT m_overheatDropMultiplier;					// 0xE0
		INT m_primaryAmmoToFill;						// 0xE4
		WeakPtr<Entity> m_characterMeleeEntity;			// 0xE8
		INT m_externalPrimaryMagazineCapacity;			// 0xEC
		Function m_primaryFire;							// 0xF0
		WeaponModifier* m_weaponModifier;				// 0x128
		DWORD m_activeContext;							// 0x12C
		DWORD m_additionMagazines;						// 0x130
		DWORD m_impulseDelay;							// 0x134
	}; // 0x138
	class Tool
		: public ITypedObject			// 0x00
	{
	public:
		WeaponData* m_firingData;		// 0x04
		CHAR m_isPlayerControlled;		// 0x08
		PAD(0x3);						// 0x09
	}; // 0xC
	class ToolData
		: public DataContainer		// 0x00
	{
	public:
		INT m_isAlwaysActive;		// 0x08
	}; // 0x0C
	class WeaponData
		: public ToolData					// 0x00
	{
	public:
		INT m_showLaserPaintedVehicles;		// 0x0C
	}; // 0x10
	class FiringDispersion
	{
	public:
		FLOAT m_currAngle;				// 0x00
		FLOAT m_multiplier;				// 0x04
		FiringDispersionData* m_data;	// 0x08
		PAD(0x10);						// 0x0C
	}; // 0x1C
	class Weapon
		: public Tool												// 0x00
	{
	public:
		WeaponFiringData* m_firingData;								// 0x0C
		WeaponModifier* m_modifier;									// 0x10
		PAD(0xC);													// 0x14
		Vec3 m_moveSpeed;											// 0x20
		D3DXMATRIX m_shootSpace;								// 0x30
		LinearTransform m_shootSpaceDelta;							// 0x70
		eastl::fixed_vector<FiringDispersion, 3, 2> m_dispersion;	// 0xB0
		FLOAT m_externalDispersionAngle;							// 0x118
		CHAR m_currPose;											// 0x11C
		CHAR m_useSecondaryProjectile;								// 0x11D
		PAD(0x2);													// 0x11E
	}; // 0x120
	class RayCastHit
	{
	public:
		Vec3 m_position;					// 0x00
		Vec3 m_normal;						// 0x10
		PhysicsEntityBase* m_rigidBody;		// 0x20
		MaterialContainerPair* m_material;	// 0x24
		DWORD m_part;						// 0x28
		INT m_bone;							// 0x2C
		FLOAT m_lambda;						// 0x30
		PAD(0xC);							// 0x34
	}; // 0x40
	class ControllableFinder
	{
	public:
		PAD(0x468);												// 0x00
		GameWorld* m_gameWorld;									// 0x468
		WeakPtr<ClientPlayer> m_ignoredPlayer;					// 0x46C
		WeakPtr<ClientControllableEntity> m_controllableInAim;	// 0x470
		PAD(0xC);												// 0x474
		Vec3 m_lastRayBegin;									// 0x480
		Vec3 m_lastRayEnd;										// 0x490
		RayCastHit m_lastHIt;									// 0x4A0
	}; // 0x4E0
	class EntityCollection
	{
	public:
		EntityCollectionSegment* firstSegment;	// 0x00
		EntityCreator* creator;					// 0x04
	}; // 0x08
	class EntityWorld
	{
	public:
		class SpatialSize
		{
		public:
			FLOAT halfSizeXZ;	// 0x00
			FLOAT minY;			// 0x04
		}; // 0x08
		class RemovedEntityInfo
		{
		public:
			Entity* entity;				// 0x00
			LPVOID func;				// 0x04
			EntityCreator* creator;		// 0x08
			LPVOID userData;			// 0x0C
		}; // 0x10
		class EntityCollection
		{
		public:
			EntityCollectionSegment* firstSegment;	// 0x00
			EntityCreator* creator;					// 0x04
		}; // 0x08

		//LPVOID vftable;														// 0x00
		SpatialSize m_spatialSize;											// 0x04
		eastl::fixed_vector<RemovedEntityInfo, 128, 2> m_removedEntities;	// 0x0C
		eastl::vector<fb::EntityCollection> m_collections;						// 0x820
		SubLevel* m_rootLevel;												// 0x830
		WORD m_entityRuntimeId;												// 0x834
		PAD(0x2);															// 0x836
		CHAR m_isDeletingAllEntities;										// 0x838
		CHAR m_isLoadingSaveGame;											// 0x839
		PAD(0x2);															// 0x83A
	}; // 0x840
	class EntityCreator
	{
	public:
		LPVOID vftable;						// 0x00
		EntityCreator* m_previousCreator;	// 0x04
		EntityCreator* m_nextCreator;		// 0x08
		Realm m_realm;						// 0x0C
		INT m_linked;						// 0x10
	}; // 0x14
	class ClientGrenadeEntity
		: public ClientGameEntity
	{
	public:
		char pad[536];                                    //0020
		fb::WeakPtr<fb::ClientSoldierEntity> m_shooter;    //0238

		__forceinline int GetTeamID(void)
		{
			if (m_shooter.GetData())
			{
				fb::ClientSoldierEntity* pShooter = reinterpret_cast< fb::ClientSoldierEntity* >(m_shooter.GetData());
				if (pShooter)
				{
					fb::ClientPlayer* pShooterI = pShooter->m_pPlayer;
					if (pShooterI)
						return pShooterI->m_teamId;
				}
			}
			return 0;
		}
		__forceinline fb::ClientSoldierEntity* GetShooterSoldier(void)
		{
			if (m_shooter.GetData())
			{
				fb::ClientSoldierEntity* pShooter = reinterpret_cast< fb::ClientSoldierEntity* >(m_shooter.GetData());
				if (pShooter)
					return pShooter;
			}
			return 0;
		}
		__forceinline ClientPlayer* GetShooterPlayer(void)
		{
			if (m_shooter.GetData())
			{
				fb::ClientSoldierEntity* pShooter = reinterpret_cast< fb::ClientSoldierEntity* >(m_shooter.GetData());
				if (pShooter)
					return pShooter->m_pPlayer;
			}
			return 0;
		}
	};
	class ClientExplosionPackEntity
		//     : public ClientGameEntity
	{
	public:
		//char pad[656];        //0020
		//int teamid;            //02B0
		PAD(0x2D4);
		int m_teamId; // this+0x290
		int m_damageGiverPlayerId; // this+0x294
		bool m_isSpotted; // this+0x298
	};
	class SoldierWeaponBlueprint
	{
	public:
		char unknown0[8];    //0000
		fb::String m_name;    //0008

		__forceinline char *GetWeaponName()
		{
			static char szWeaponName[20] = { 0 };
			bool asdf = false;

			char *name = m_name.GetString();
			//MessageBoxA(0,name,0,0);

			if (name)

			{
				for (int i = 0, j = 0; i < (int)strlen(name); i++)
				{
					if (asdf && name[i] == '/')
					{
						szWeaponName[j] = 0;
						break;
					}

					if (name[i] == '/' && !asdf)
					{
						asdf = true;
						continue;
					}

					if (asdf)
					{
						szWeaponName[j] = name[i];
						j++;
					}
				}
			}
			return szWeaponName;
		}
	};
	class PickupEntity
	{
	public:
		class WeaponInfo
		{
		public:
			fb::SoldierWeaponBlueprint *blueprint;    //0000
			unsigned char pad[132];                    //0004
			unsigned int weaponSlot;                //0088
			int altWeaponSlot;                        //008C
			int linkedToWeaponSlot;                    //0090
			int clipAmmo;                            //0094
			int spareAmmo;                            //0098
			float reloadTimer;                        //009C
			bool ammoPickup;                        //00A0
		};

		void *m_pickupEntityData;                                    //0000
		eastl::vector< fb::PickupEntity::WeaponInfo * > m_weapons;    //0004
		int m_droppedByTeam;                                        //0014
		void *dummy;//class fb::SmartRef<fb::CharacterCustomizationAsset const > m_droppedByCharacterCustomizationAsset;                     //0018

	};
	class ClientPickupEntity
		: public ClientGameEntity
	{
	public:
		char padding001[140];                        //0020
		fb::PickupEntity::WeaponInfo *pWeaponInfo;    //00AC
	};
	enum RayCastFlags
	{
		CheckDetailMesh = 0x0DB,
		IsAsyncRaycast = 0x2,
		DontCheckWater = 0x0FD,
		DontCheckTerrain = 0x07A,
		DontCheckRagdoll = 0x10,
		DontCheckCharacter = 0x20,
		DontCheckGroup = 0x40,
		DontCheckPhantoms = 0x80,
	};
	class IPhysicsRayCaster
	{
	public:
		virtual bool physicsRayQuery(const char *, D3DXVECTOR4 &, D3DXVECTOR4 &, RayCastHit &, unsigned int, void* PhysicsEntityList); //0x00
		virtual void *asyncPhysicsRayQuery(const char *ident, D3DXVECTOR4 *from, D3DXVECTOR4 *to, unsigned int flags, void *excluded); //0x00
	}; // 0x04
	class GameWorld
	{
	public:
		virtual void Function0();
		virtual void Function1();
		virtual void Function2();

		char _0x0000[2076];
		eastl::vector< fb::EntityCollection > m_collections; //0x0820
		char _0x0830[16];
		fb::physics::IPhysicsRayCaster m_rayCaster; //0x0840
	};
	class ObjectBlueprint
		: public Blueprint			// 0x00
	{
	public:
		DataContainer* m_object;	// 0x20
	}; // 0x24
	class ProjectileBlueprint
		: public ObjectBlueprint	// 0x00
	{
	}; // 0x24
	class WeaponEntityData
	{
	public:
	};
	class ProjectileSyncInfo
	{
	public:
		void* projectile;                     // this+0x0
		void* projectileData;                     // this+0x4
		bool localPlayer;                     // this+0x8
		void* shooterArg;                     // this+0xC
		fb::LinearTransform shootSpace;                     // this+0x10
		fb::LinearTransform visualShootSpace;                     // this+0x50
		fb::Vec3 initialSpeed;                     // this+0x90
		unsigned int randomSeed;                     // this+0xA0
		bool trace;                     // this+0xA4
		fb::Vec3 * targetPosition;                     // this+0xA8
		void* targetObject;                     // this+0xAC
		float damageMultiplier;                     // this+0xB0
		float explosionDamageMultiplier;                     // this+0xB4
		void* weaponUnlockAsset;                     // this+0xB8
	};
	class ClientWeapon
	{
	public:
		char _0x0000[8];
		WeaponEntityData* m_data; //0x0008
		WeaponFiringData* m_firingData; //0x000C
		WeaponModifier* m_weaponModifier; //0x0010
		//
		DWORD m_zoomLevel;														// 0x1C8
		CHAR m_zoomLevelLocked;													// 0x1CC
		CHAR m_playFire;														// 0x1CD
		CHAR m_stopFire;														// 0x1CE
		PAD(0x1);
		//
		ControllableFinder m_controllableFinder;                                 // 0x1D0
		char _0x0014[1796];
		eastl::vector< ProjectileSyncInfo > m_bulletsToSpawnOnSyncUpdate; //0x0718
		char _0x0728[280];

	};//Size=0x0840
	class VehicleEntity
	{
	public:
		DWORD xz;
		PhysicsEntity* m_physicsEntity;  // 0x00
		//FLOAT m_waterLevel;     // 0x04
		FLOAT m_terrainLevel;    // 0x08
		FLOAT m_waterLevelUpdateTimer;  // 0x0C
		FLOAT m_terrainLevelUpdateTimer; // 0x10
	}; // 0x14
	class DynamicBitSet
	{
	public:
		LPVOID data;		// 0x00
		INT bitCount;		// 0x04
	}; // 0x08
	class PhysicsEntityParts
	{
	public:
		DynamicBitSet m_enabled;			// 0x00
		D3DXMATRIX* m_base;			// 0x08
		EntityBusPeer** m_transformNodes;	// 0x0C
		hkpShape** m_detailShapes;			// 0x10
		LPBYTE m_transformIndices;			// 0x14
	}; // 0x18

	template <class T>

	class TntObject
		: public T			// 0x00
	{
	public:
		UINT m_refCnt;		// sizeof(T) + 0x00
	}; // sizeof(T) + 0x04
	class IResourceObject
	{
	public:
		LPVOID vftable;		// 0x00
	}; // 0x04
	class HavokPhysicsData
		: public TntObject<IResourceObject>
	{
	public:
		class LoadedData
		{
		public:
			DWORD partCount;							// 0x00
			RelocArray<Vec3> partTranslations;			// 0x04
			PAD(0x4);									// 0x0C
			RelocArray<AxisAlignedBox> localAabbs;		// 0x10
			PAD(0x4);									// 0x18
			RelocArray<BYTE> materialIndices;			// 0x1C
			PAD(0x4);									// 0x24
			RelocArray<UINT> materialFlagsAndIndices;	// 0x28
			FLOAT scale;								// 0x34
			BYTE materialCountUsed;						// 0x38
			BYTE highestMaterialIndex;					// 0x39
			PAD(0x2);									// 0x3A
		}; // 0x3C

		MemoryArena* m_arena;				// 0x08
		LoadedData* m_loadedData;			// 0x0C
		LPVOID m_nativeData;				// 0x10
		INT m_nativeDataSize;				// 0x14
		DWORD m_rootContainer;				// 0x18
		LPUINT m_materialFlagsAndIndices;	// 0x1C
		UINT m_runtimeFlags;				// 0x20
	}; // 0x24
	class PhysicsEntity
		: public PhysicsEntityBase		// 0x00
	{
	public:
		PAD(0x8);								// 0x58
		AxisAlignedBox m_rbAabb;				// 0x60
		PhysicsEntityParts m_parts;				// 0x80
		PhysicsEntityData* m_data;				// 0x98
		HavokPhysicsData* m_havokPhysicsPart;	// 0x9C
		DWORD m_rb;								// 0xA0 hkpRigidBody*
		hkpShape* m_raycastShape;				// 0xA4
		DWORD m_rbProxy;						// 0xA8 hkpRigidBody*
		DWORD m_characterBody;					// 0xAC hkpRigidBody*
		INT m_simulationType;					// 0xB0
	}; // 0xB4
	class HavokAsset
		: public Asset								// 0x00
	{
	public:
		FLOAT m_scale;								// 0x0C
		RefArray<DataContainer> m_externalAssets;	// 0x10
	}; // 0x14
	class PhysicsEntityData
		: public EntityData						// 0x00
	{
	public:
		PAD(0x4);								// 0x0C
		Vec3 m_intertiaModifier;				// 0x10
		RefArray<HavokAsset> m_scaledAssets;	// 0x20
		RefArray<DWORD> m_rigidBodies;			// 0x24
		DataContainer* m_asset;					// 0x28
		DataContainer* m_floatPhysics;			// 0x2C
		FLOAT m_mass;							// 0x30
		FLOAT m_restitution;					// 0x34
		FLOAT m_friction;						// 0x38
		FLOAT m_linearVelocityDamping;			// 0x3C
		FLOAT m_angularVelocityDamping;			// 0x40
		DataContainer* m_proximity;				// 0x44
		RefArray<DWORD> m_constraints;			// 0x48
		CHAR m_encapsulatePartsInLists;			// 0x4C
		CHAR m_movableParts;					// 0x4D
		PAD(0x2);								// 0x4E
	}; // 0x50
	class ClientVehicleEntity
		: public ClientControllableEntity,								// 0x00
		public network::Interpolator<LPVOID>,							// 0xB4
		public VehicleEntity											// 0xF4
	{
	public:
		class LockableCallback
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04

		PAD(0x8);														// 0x108
		AxisAlignedBox m_childrenAabb;									// 0x110
		Vec3 m_dirtColor;												// 0x130
		Vec3 m_prevSpeed;												// 0x140 VehicleSpeed
		Vec3 m_prev2Speed;												// 0x150
		FLOAT m_prevDeltaTime;											// 0x160
		network::ClientNetworkableGroup m_networkableGroup;				// 0x164
		ClientChassisComponent* m_chassis;								// 0x1FC
		ClientVehicleEntityHealth* m_vehicleHealth;						// 0x200
		WeakPtr<ClientPlayer> m_remoteControlledDamageGiverPlayer;		// 0x204
		DWORD m_meshModel;												// 0x208 MeshModel*
		DWORD m_cockpitMeshModel;										// 0x20C MeshModel*
		DWORD m_vehicleSound;											// 0x210 VehicleSound*
		LockableCallback* m_lockableCallback;							// 0x214
		DWORD m_upgradableCallback;										// 0x218
	};

	template <class T>

	class PartComponent
		: public T							// 0x00
	{
	public:
		PhysicsEntity* m_physicsEntity;		// sizeof(T) + 0x00
		CHAR m_isNetworkable;				// sizeof(T) + 0x04
		PAD(0x7);							// sizeof(T) + 0x05
		INT m_healthStateIndex;				// sizeof(T) + 0x0C
	}; // sizeof(T) + 0x10
	class HealthStateEntityManager
	{
	public:
		LPVOID vftable;							// 0x00
		GameWorld* m_world;						// 0x04
		DWORD m_masterManager;					// 0x08
		PartComponentData* m_partComponent;		// 0x0C
		DWORD m_instanceId;						// 0x10
	}; // 0x14
	class PartComponentData
		: public ComponentData		// 0x00
	{
	public:
		PAD(0x8);					// 0x58
		DWORD m_healthStates;		// 0x60 RefArray<HealthStateData>
		DWORD m_partLinks;			// 0x64 RefArray<PartLinkData>
		CHAR m_isSupported;			// 0x68
		CHAR m_isFragile;			// 0x69
		CHAR m_isNetworkable;		// 0x6A
		CHAR m_isWindow;			// 0x6B
		CHAR m_animatePhysics;		// 0x6C
		PAD(0x3);					// 0x6D
	}; // 0x70
	class ClientHealthStateEntityManager
		: public HealthStateEntityManager	// 0x00
	{
	public:
	}; // 0x14
	class ClientPartComponent
		: public PartComponent<ClientComponent>,						// 0x00
		public network::IClientNetworkableGroupMember					// 0x20
	{
	public:
		DWORD m_meshModel;												// 0x28 MeshModel*
		DWORD m_interpolationObject;									// 0x2C ClientPartComponent::InterpolationObject*
		PAD(0x4);														// 0x30
		ClientHealthStateEntityManager m_healthStateEntityManager;		// 0x34
		PAD(0x8);														// 0x48
	}; // 0x50
	class ChassisComponent
	{
	public:
		LPVOID vftable;							// 0x00
		PAD(0xC);								// 0x04
		Vec3 m_initialWorldPosition;			// 0x10
		PhysicsEntity* m_physics;				// 0x20
		DWORD m_vehiclePhysics;					// 0x24 vehicle::IVehicle (vftable)
		DWORD m_gearboxPhysics;					// 0x28 vehicle::IGearbox (vftable)
		FLOAT m_timeDisabledByEMP;				// 0x2C
		LinearTransform m_localTransform;		// 0x30
		DWORD m_inputModifierIndex;				// 0x70
		DWORD m_pPlayerCount;					// 0x74
		FLOAT m_inputSwayTimer;					// 0x78
		FLOAT m_inputSwayYaw;					// 0x7C
		FLOAT m_inputSwayPitch;					// 0x80
		FLOAT m_inputSwayRoll;					// 0x84
		DWORD m_landingGears;					// 0x88
	}; // 0x8C
	class ClientChassisComponent
		: public ClientPartComponent,							// 0x00
		public ChassisComponent									// 0x50
	{
	public:
		class EffectInfo
		{
		public:
			Vec3 position;			// 0x00
			Vec3 normal;			// 0x10
			Asset* effectAsset;		// 0x20
		}; // 0x24

		PAD(0x4);												// 0xDC
		AxisAlignedBox m_cachedBox;								// 0xE0
		Vec3 m_linearVelocity;									// 0x100
		Vec3 m_angularVelocity;									// 0x110
		Vec3 m_prevLinearVelocity;								// 0x120
		DWORD m_chassisEffectHandle;							// 0x130
		DWORD m_effectCount;									// 0x134
		PAD(0x8);												// 0x138
		EffectInfo m_effects;									// 0x140
		PAD(0xEC);												// 0x164
		eastl::vector<UINT> m_waterEffectHandles;				// 0x250
		eastl::vector<UINT> m_waterStreakEffectHandles;			// 0x260
		ClientChassisComponentSimulation* m_controller;			// 0x270
		ClientChassisComponentReplication* m_replicatedBody;		// 0x274
		ClientChassisComponentPrediction* m_predictedBody;		// 0x278
		DWORD m_updater;										// 0x27C ClientChassisComponent::PredictionUpdater
		DWORD m_repUpdater;										// 0x280 ClientChassisComponent::ReplicationUpdater
		MaterialContainerPair* m_groundMaterial;				// 0x284
		DWORD m_speedField;										// 0x28C
		FLOAT m_gForce;											// 0x290
	}; // 0x164
	class CollisionState
	{
	public:
		Vec3 m_position;			// 0x00
		Vec3 m_normal;				// 0x10
		UINT m_materialIndex;		// 0x20
		UINT m_ownMaterialIndex;	// 0x24
		FLOAT m_speed;				// 0x28
		INT m_valid;				// 0x2C
	}; // 0x30
	class ClientChassisComponentReplicationState
	{
	public:
		Vec3 m_position;						// 0x00
		Vec3 m_orientation;					// 0x10
		Vec3 m_velocity;						// 0x20
		Vec3 m_angularVelocity;				// 0x30
		fb::CollisionState m_collision;		// 0x40
	}; // 0x70
	class ClientChassisComponentSimulation
	{
	public:
		LPVOID vftable;			// 0x00
	}; // 0x04
	class ClientChassisComponentReplication
		: public ClientChassisComponentSimulation		// 0x00
	{
	public:
		class BodyInfo
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04

		class Interpolator
			: public network::Interpolator<ClientChassisComponentReplicationState>		// 0x00
		{
		public:
			ClientChassisComponentReplication* m_replication;							// 0x40
		}; // 0x44

		BodyInfo* m_info;								// 0x04
		Interpolator* m_interpolationObject;			// 0x08
		FLOAT m_interpolationFactor;					// 0x0C
		INT m_hasLowVelocityTimer;						// 0x10
		FLOAT m_aheadOfTime;							// 0x14
	}; // 0x18
	class RigidBodyState
	{
	public:
		Vec3 m_orientation;			// 0x00
		Vec3 m_position;			// 0x10
		Vec3 m_linearVelocity;		// 0x20
		Vec3 m_angularVelocity;		// 0x30
		CHAR m_isSleeping;			// 0x40
		PAD(0x3);					// 0x41
	}; // 0x44
	class FloatPhysicsState
	{
	public:
		LPVOID vftable;		// 0x00
	}; // 0x04
	class ClientChassisComponentPrediction
		: public ClientChassisComponentSimulation		// 0x00
	{
	public:
		class State
		{
		public:
			RigidBodyState rigidBodyState;			// 0x00
			PAD(0xC);								// 0x44
			FloatPhysicsState floatPhysicsState;	// 0x50
			DWORD vehicleState;						// 0x54 vehicle::VehicleState
			DWORD gearboxState;						// 0x58 vehicle::GearboxState
			DWORD aerodynamicPhysicsState;			// 0x5C vehicle::AeroDynamicPhysicsState
		}; // 0x60

		class CorrectionInterpolationDelta
		{
		public:
			Vec3 deltaTrans;			// 0x00
			Vec3 deltaOrientation;		// 0x10
		}; // 0x20

		class Updater
		{
		public:
			LPVOID vftable;		// 0x00
		}; // 0x04

		State* m_currentState;							// 0x04
		PAD(0x8);										// 0x08
		State m_predictionState;						// 0x10
		State m_correctionState;						// 0x70
		RigidBodyState m_prevRigidBodyState;			// 0xD0
		PAD(0xC);
		CorrectionInterpolationDelta m_correctionInterpolationDelta;
		CorrectionInterpolationDelta m_frameCorrectionDelta;
		FLOAT m_correctionInterpolationTimer;
		FLOAT m_correctionInterpolationTime;
		FLOAT m_frameInterpolationFactor;
		Updater* m_updater;
	};
	class VehicleEntityHealth
	{
	public:
		class VehicleEntityHealthZone
		{
		public:
			VehicleHealthZoneData* data;		// 0x00
			FLOAT health;						// 0x04
			FLOAT shieldHealth;					// 0x08
			CHAR useProtectedShields;			// 0x0C
			PAD(0x3);							// 0x0D
		}; // 0x10

		LPVOID vftable;											// 0x00
		eastl::vector<Component*> m_components;					// 0x04
		eastl::vector<VehicleEntityHealthZone> m_healthZones;	// 0x14
	}; // 0x24
	class VehicleHealthZoneData
	{
	public:
		FLOAT m_maxHealth;					// 0x00
		FLOAT m_maxShieldHealth;			// 0x04
		FLOAT m_damageAngleMultiplier;		// 0x08
		FLOAT m_minDamageAngle;				// 0x0C
		INT m_useDamageAngleCalculation;	// 0x10
	}; // 0x14
	class ClientVehicleEntityHealth
		: public VehicleEntityHealth	// 0x00
	{
	}; // 0x14
	class RenderScreenInfo
	{
	public:
		UINT m_nWidth;					// this+0x0
		UINT m_nHeight;					// this+0x4
		UINT m_nWindowWidth;			// this+0x8
		UINT m_nWindowHeight;			// this+0xC
		FLOAT fRefreshRate;				// this+0x10
	};
	class Dx11Renderer
	{
	public:
		char _0x0000[16];
		HWND__* m_hwnd; //0x0010
		char _0x0014[12];
		__int32 screenWidth; //0x0020
		__int32 screenHeight; //0x0024
		char _0x0028[176];
		ID3D11Device* m_device; //0x000D8
		ID3D11DeviceContext* m_deviceContext; //0x000DC
		char _0x00F0[20];
		IDXGISwapChain* m_swapChain; //0x00F4

	public:
		static Dx11Renderer* Singleton()
		{
			return *(Dx11Renderer**)(OFFSET_DXRENDERER);
		}
	};
	class DxRenderer
	{
	public:
		BYTE Pad_000[0x8];				// 0x00
		UINT m_nFrameCounter;			// 0x08
		BOOL m_bFrameInProgress;		// 0x0C
		HWND m_hWnd;					// 0x10
		BYTE Pad_014[0x4];				// 0x14
		BYTE m_bFullscreenWanted;		// 0x18
		BYTE m_bFullscreenActive;		// 0x19
		BYTE m_bMinimized;				// 0x1A
		BYTE m_bMinimizing;				// 0x1B
		BYTE m_bResizing;				// 0x1C
		BYTE m_bOccluded;				// 0x1D
		BYTE m_bVSync;					// 0x1E
		PAD(0x1);						// 0x1F
		RenderScreenInfo m_screenInfo;	// 0x20
		PAD(0xA4);						// 0x34
		ID3D11Device* pDevice;			// 0xD8
		ID3D11DeviceContext* pContext;  // 0xDC
		PAD(0x14);						// 0xE0
		IDXGISwapChain* pSwapChain;		// 0xF4

	public:
		static DxRenderer* Singleton()
		{
			return *(DxRenderer**)OFFSET_DXRENDERER;
		}
	};
	class RenderViewDesc
	{
	public:
		LinearTransform transform;				// 0x00
		INT type;								// 0x40
		PAD(0x4);								// 0x44
		FLOAT fovY;								// 0x48
		FLOAT defaultFovY;						// 0x4C
		FLOAT nearPlane;						// 0x50
		FLOAT farPlane;							// 0x54
		FLOAT aspect;							// 0x58
		FLOAT orthoWidth;						// 0x5C
		FLOAT orthoHeight;						// 0x60
		FLOAT stereoSeparation;					// 0x64
		FLOAT stereoConvergence;				// 0x68
		Vec2 viewportOffset;					// 0x6C
		Vec2 viewportScale;						// 0x74
	};
	class RenderView
	{
	public:
		RenderViewDesc m_desc;							// 0x00
		PAD(0x4);										// 0x7C
		INT m_dirtyFlags;								// 0x80
		PAD(0x16C);										// 0x84
		FLOAT m_fovX;									// 0x1F0
		FLOAT m_depthToWidthRatio;						// 0x1F4
		FLOAT m_fovScale;								// 0x1F8
		FLOAT m_fovScaleSqr;							// 0x1FC
		LinearTransform m_viewMatrix;					// 0x200
		LinearTransform m_viewMatrixTranspose;			// 0x240
		LinearTransform m_viewMatrixInverse;			// 0x280
		LinearTransform m_projectionMatrix;				// 0x2C0
		LinearTransform m_viewMatrixAtOrigin;			// 0x300
		LinearTransform m_projectionMatrixTranspose;	// 0x340
		LinearTransform m_projectionMatrixInverse;		// 0x380
		//D3DXMATRIX m_viewProjectionMatrix;			// 0x3C0
		fb::Mat4 m_viewProjectionMatrix;			// 0x3C0
		LinearTransform m_viewProjectionMatrixTranspose;// 0x400
		LinearTransform m_viewProjectionMatrixInverse;	// 0x440

	public:
		BOOL Update()
		{
			DxRenderer* dxRenderer = DxRenderer::Singleton();
			if (dxRenderer == NULL)
				return FALSE;

			FLOAT screenX = static_cast<FLOAT>(dxRenderer->m_screenInfo.m_nWindowWidth);
			FLOAT screenY = static_cast<FLOAT>(dxRenderer->m_screenInfo.m_nWindowHeight);
			this->m_desc.aspect = screenX / screenY;

			((VOID(__fastcall *)(RenderView*, LPVOID))OFFSET_UPDATEMATRICES)(this, NULL);
			return TRUE;
		}
	};
	class WeaponFiringUpdateContext
	{
	public:
		char _0x0000[32];
		float damageMultiplier; //0x0020
		char _0x0024[12];
		unsigned int ticks; //0x0030

	};//Size=0x0034
	class Deviation
	{
	public:
		float m_pitch; //0x0000
		float m_yaw; //0x0004
		float m_roll; //0x0008
		float m_transY; //0x000C

	};//Size=0x0010
	class Random
	{
	public:
		unsigned int m_value; //0x0000
		unsigned int m_tableIndex; //0x0004
		float m_nextNormal; //0x0008
		bool m_nextNormalIsValid; //0x000C
		char _0x000D[3];

	};//Size=0x0010
	class GunSway
	{
	public:
		virtual void Function0(); //
		virtual void Function1(); //
		virtual void Function2(); //
		virtual void primaryFireShotSpawnedCallback(float, bool, WeaponFiringUpdateContext*);; //
		virtual void Function4(); //
		virtual void Function5(); //
		virtual void Function6(); //
		virtual void Function7(); //
		virtual void Function8(); //
		virtual void Function9(); //

		char _0x0004[168];
		Deviation m_currentRecoilDeviation; //0x00AC
		char _0x00BC[100];
		float m_dispersionAngle; //0x0120
		float m_minDispersionAngle; //0x0124
		float m_crossHairDispersionFactor; //0x0128
		Deviation m_currentDispersionDeviation; //0x012C
		float m_currentGameplayDeviationScaleFactor; //0x013C
		float m_currentVisualDeviationScaleFactor; //0x0140
		char _0x0144[4];
		Random m_random; //0x0148
		unsigned int m_seed; //0x0158
		float m_randomAngle; //0x015C
		float m_randomRadius; //0x0160
		char _0x0164[32];

	};//Size=0x0184
	class GameRenderViewParams
	{
	public:
		RenderView view;						// 0x00
		RenderView prevView;					// 0x480
		RenderView secondaryStreamingView;		// 0x900
		INT secondaryStreamingViewEnable;		// 0xD80
		PAD(0xC);								// 0xD84
		D3DXMATRIX firstPersonTransform;		// 0xD90
	};
	class AbilityCallback
	{
	public:
		virtual void Function0(); //

	};//Size=0x0004
	class SuppressionCallback
	{
	public:
		virtual void Function0(); //

	};//Size=0x0004
	class IGameRenderer
	{
	public:
		LPVOID vftable;		// 0x00
		INT m_refCount;		// 0x04
	}; // 0x08
	class GameRenderer :
		public IGameRenderer
	{
	public:
		PAD(0x48);							// 0x08
		GameRenderViewParams m_viewParams;	// 0x50

	public:
		static GameRenderer* Singleton()
		{
			return *(GameRenderer**)(OFFSET_GAMERENDERER);
		}
	};
	enum InputConceptIdentifiers
	{
		ConceptMoveFB = 0x00,
		ConceptMoveLR = 0x01,
		ConceptMoveForward = 0x02,
		ConceptMoveBackward = 0x03,
		ConceptMoveLeft = 0x04,
		ConceptMoveRight = 0x05,
		ConceptYaw = 0x06,
		ConceptPitch = 0x07,
		ConceptRoll = 0x08,
		ConceptRecenterCamera = 0x09,
		ConceptFire = 0x0A,
		ConceptAltFire = 0x0B,
		ConceptFireCountermeasure = 0x0C,
		ConceptReload = 0x0D,
		ConceptZoom = 0x0E,
		ConceptToggleCamera = 0x0F,
		ConceptSprint = 0x010,
		ConceptCrawl = 0x011,
		ConceptToggleWeaponLight = 0x012,
		ConceptJump = 0x013,
		ConceptCrouch = 0x014,
		ConceptCrouchOnHold = 0x015,
		ConceptProne = 0x016,
		ConceptInteract = 0x017,
		ConceptPickUp = 0x018,
		ConceptDrop = 0x019,
		ConceptBreathControl = 0x01A,
		ConceptParachute = 0x01B,
		ConceptSwitchInventoryItem = 0x01C,
		ConceptSelectInventoryItem1 = 0x01D,
		ConceptSelectInventoryItem2 = 0x01E,
		ConceptSelectInventoryItem3 = 0x01F,
		ConceptSelectInventoryItem4 = 0x020,
		ConceptSelectInventoryItem5 = 0x021,
		ConceptSelectInventoryItem6 = 0x022,
		ConceptSelectInventoryItem7 = 0x023,
		ConceptSelectInventoryItem8 = 0x024,
		ConceptSelectInventoryItem9 = 0x025,
		ConceptSwitchToPrimaryWeapon = 0x026,
		ConceptSwitchToGrenadeLauncher = 0x027,
		ConceptSwitchToStaticGadget = 0x028,
		ConceptSwitchToDynamicGadget1 = 0x029,
		ConceptSwitchToDynamicGadget2 = 0x02A,
		ConceptMeleeAttack = 0x02B,
		ConceptThrowGrenade = 0x02C,
		ConceptCycleFireMode = 0x02D,
		ConceptChangeVehicle = 0x02E,
		ConceptBrake = 0x02F,
		ConceptHandBrake = 0x030,
		ConceptClutch = 0x031,
		ConceptGearUp = 0x032,
		ConceptGearDown = 0x033,
		ConceptGearSwitch = 0x034,
		ConceptNextPosition = 0x035,
		ConceptSelectPosition1 = 0x036,
		ConceptSelectPosition2 = 0x037,
		ConceptSelectPosition3 = 0x038,
		ConceptSelectPosition4 = 0x039,
		ConceptSelectPosition5 = 0x03A,
		ConceptSelectPosition6 = 0x03B,
		ConceptSelectPosition7 = 0x03C,
		ConceptSelectPosition8 = 0x03D,
		ConceptCameraPitch = 0x03E,
		ConceptCameraYaw = 0x03F,
		ConceptMapZoom = 0x040,
		ConceptMapInnerZoom = 0x041,
		ConceptMapSize = 0x042,
		ConceptMapThreeDimensional = 0x043,
		ConceptScoreboard = 0x044,
		ConceptMenu = 0x045,
		ConceptSpawnMenu = 0x046,
		ConceptCancel = 0x047,
		ConceptCommMenu1 = 0x048,
		ConceptCommMenu2 = 0x049,
		ConceptCommMenu3 = 0x04A,
		ConceptAccept = 0x04B,
		ConceptDecline = 0x04C,
		ConceptSelect = 0x04D,
		ConceptBack = 0x04E,
		ConceptActivate = 0x04F,
		ConceptDeactivate = 0x050,
		ConceptEdit = 0x051,
		ConceptView = 0x052,
		ConceptParentNavigateLeft = 0x053,
		ConceptParentNavigateRight = 0x054,
		ConceptMenuZoomIn = 0x055,
		ConceptMenuZoomOut = 0x056,
		ConceptPanX = 0x057,
		ConceptPanY = 0x058,
		ConceptVoiceFunction1 = 0x059,
		ConceptSayAllChat = 0x05A,
		ConceptTeamChat = 0x05B,
		ConceptSquadChat = 0x05C,
		ConceptSquadLeaderChat = 0x05D,
		ConceptQuicktimeInteractDrag = 0x05E,
		ConceptQuicktimeFire = 0x05F,
		ConceptQuicktimeBlock = 0x060,
		ConceptQuicktimeFastMelee = 0x061,
		ConceptQuicktimeJumpClimb = 0x062,
		ConceptQuicktimeCrouchDuck = 0x063,
		ConceptFreeCameraMoveUp = 0x064,
		ConceptFreeCameraMoveDown = 0x065,
		ConceptFreeCameraMoveLR = 0x066,
		ConceptFreeCameraMoveFB = 0x067,
		ConceptFreeCameraRotateX = 0x068,
		ConceptFreeCameraRotateY = 0x069,
		ConceptFreeCameraIncreaseSpeed = 0x06A,
		ConceptFreeCameraDecreaseSpeed = 0x06B,
		ConceptFreeCameraFOVModifier = 0x06C,
		ConceptFreeCameraChangeFOV = 0x06D,
		ConceptFreeCameraSwitchSpeed = 0x06E,
		ConceptFreeCameraTurboSpeed = 0x06F,
		ConceptFreeCameraActivator1 = 0x070,
		ConceptFreeCameraActivator2 = 0x071,
		ConceptFreeCameraActivator3 = 0x072,
		ConceptFreeCameraMayaInputActivator = 0x073,
		ConceptTargetedCameraDistance = 0x074,
		ConceptTargetedCameraRotateX = 0x075,
		ConceptTargetedCameraRotateY = 0x076,
		ConceptTargetedCameraChangeSpeed = 0x077,
		ConceptLThumb = 0x078,
		ConceptRThumb = 0x079,
		ConceptUndefined = 0x07A,
		ConceptSize = 0x07B
	};
	class BorderInputNode
	{
	public:
		virtual void Function0();
		virtual void Function1();
		virtual void Function2();
		virtual void Function3();
		virtual void Function4();
		virtual void Function5();
		virtual void Function6();
		virtual void Function7();
		virtual void Function8();
		virtual void Function9();
		virtual void Function10();
		virtual void Function11();
		virtual void Function12();
		virtual void Function13();
		virtual void Function14();
		virtual void Function15();
		virtual void Function16();
		virtual void Function17();
		virtual void Function18();
		virtual void Function19();
		virtual void Function20();
		virtual void Function21();
		virtual void Function22();
		virtual void Function23();
		virtual void Function24();
		virtual void Function25();
		virtual void Function26();
		virtual int preFrameUpdate(float fDeltaTime);

		char _0x0004[4];
		InputCache* m_inputCache;

	public:
		static BorderInputNode* Singleton()
		{
			return *(BorderInputNode**)OFFSET_BorderInputNode;
		}
	};
	class InputCache
	{
	public:
		char _0x0000[4];
		float flInputBuffer[123];
	};

	static bool IsValid(void* ptr);
	static bool IsEnemy(fb::ClientPlayer* pPlayer);

	class __declspec(align(16)) SoldierWeaponData
	{
	public:
		PAD(0x110);
		fb::String m_damageGiverName;//0x110
		PAD(0x8);
		fb::SoldierAimingSimulationData* m_aimingController;//0x11C
		PAD(0x5C);
		fb::SoldierWeaponBlueprint* m_soldierWeaponBlueprint;//0x17C
		fb::String m_persistenceId;//0x180
		__int32 m_secondaryActionInputAction;//0x18C
		PAD(0x4);
		float m_weaponFloatParam;//0x194
		float m_maxRangeMeterDistance;//0x198
		bool m_hideWhenOutOfAmmo;//0x19C
		bool m_allowSwitchingToWeaponOutOfAmmo;//0x19D
		bool m_allowSwitchingToWeaponReloading;//0x19E
		bool m_switchToPrimaryWhenOutOfAmmo;//0x19F
		bool m_fireAndSwitchBackToPrevSupported;//0x1A0
		bool m_allowSwitchingToWeaponInVehicles;//0x1A1
		bool m_allowSwitchingToWeaponInWater;//0x1A2
		bool m_lowerOnOwnTeam;//0x1A3
		bool m_redeployWhenSwitchingWeaponStates;//0x1A4
		bool m_useQuickThrowOnAutomaticSwitchback;//0x1A5
		bool m_enableBreathControl;//0x1A6
		bool m_canBeInSupportedShooting;//0x1A7
		bool m_useDetailedRangeMeterQuery;//0x1A8
		bool m_isSilenced;//0x1A9
	};//Size 0x000001B0


	/*
			class __declspec(align(16)) WeaponEntityData :
		public fb::GameComponentEntityData	//0x0
	{
	public:
		fb::WeaponClassEnum m_weaponClass;//0x70
		fb::Arrray<fb::WeaponStateData> m_weaponStates;//0x74
		WeaponFiringData* m_weaponFiring;//0x78
		ZeroingWeaponData* m_customWeaponType;//0x7c
		fb::BFAIWeaponData* m_aIData;//0x80
		char _pad_0x84[0xc];	//0x84
	};// Size 0x00000090
		class __declspec(align(16)) fb::SoldierWeaponData :
		public fb::WeaponEntityData //0x00
	{
	public:
		//enumeration is wrong
	  fb::PickupSettingsData m_pickupSettings;//0x90
	  fb::Vec3 m_interactionOffset;//0xB0
	  fb::CustomizationAccessoryPivots m_customizationAccessoryPivots;//0xC0
	  fb::String m_damageGiverName;//0x110
	  fb::RefArray<fb::WeaponZoomLevelData> m_zoomLevels;//0x114
	  fb::SoldierAimingSimulationData* m_aimingController;//0x11C
		  fb::FirstPersonCameraData* m_firstPersonCamera;//0x120
		  fb::HudData m_hud;//0x124
		  fb::WeaponAnimBaseSetEnum m_animBaseSet;//0x174
		  fb::QuickThrowTypeEnum m_quickThrowType;//0x178
	  fb::SoldierWeaponBlueprint* m_soldierWeaponBlueprint;//0x17C
	  fb::String m_persistenceId;//0x180
	//  fb::RefArray<fb::SocketData> m_socketsWithoutUnlocks;//0x184
	  __int32 m_secondaryActionInputAction;//0x18C
		fb::CtrRefBase m_voiceOverInfo;//0x190
	  float m_weaponFloatParam;//0x194
	  float m_maxRangeMeterDistance;//0x198
	  bool m_hideWhenOutOfAmmo;//0x19C
	  bool m_allowSwitchingToWeaponOutOfAmmo;//0x19D
	  bool m_allowSwitchingToWeaponReloading;//0x19E
	  bool m_switchToPrimaryWhenOutOfAmmo;//0x19F
	  bool m_fireAndSwitchBackToPrevSupported;//0x1A0
	  bool m_allowSwitchingToWeaponInVehicles;//0x1A1
	  bool m_allowSwitchingToWeaponInWater;//0x1A2
	  bool m_lowerOnOwnTeam;//0x1A3
	  bool m_redeployWhenSwitchingWeaponStates;//0x1A4
	  bool m_useQuickThrowOnAutomaticSwitchback;//0x1A5
	  bool m_enableBreathControl;//0x1A6
	  bool m_canBeInSupportedShooting;//0x1A7
	  bool m_useDetailedRangeMeterQuery;//0x1A8
	  bool m_isSilenced;//0x1A9
	};//Size 0x000001B0

	*/
};

namespace fb
{
	enum BoneIds
	{
		BONE_HIPS = 0x4,
		BONE_SPINE = 0x5,
		BONE_SPINE1 = 0x6,
		BONE_SPINE2 = 0x7,
		BONE_LEFTSHOULDER = 0x8,
		BONE_LEFTARM = 0x9,
		BONE_LEFTARMROLL = 0xA,
		BONE_LEFTFOREARM = 0xB,
		BONE_LEFTFOREARMROLL = 0xC,
		BONE_LEFTFOREARMROLL1 = 0xD,
		BONE_LEFTHAND = 0xE,
		BONE_LEFTHANDMIDDLE0 = 0xF,
		BONE_LEFTHANDMIDDLE1 = 0x10,
		BONE_LEFTHANDMIDDLE2 = 0x11,
		BONE_LEFTHANDMIDDLE3 = 0x12,
		BONE_LEFTHANDMIDDLE4 = 0x13,
		BONE_LEFTHANDRING0 = 0x14,
		BONE_LEFTHANDRING1 = 0x15,
		BONE_LEFTHANDRING2 = 0x16,
		BONE_LEFTHANDRING3 = 0x17,
		BONE_LEFTHANDRING4 = 0x18,
		BONE_LEFTHANDPINKY0 = 0x19,
		BONE_LEFTHANDPINKY1 = 0x1A,
		BONE_LEFTHANDPINKY2 = 0x1B,
		BONE_LEFTHANDPINKY3 = 0x1C,
		BONE_LEFTHANDPINKY4 = 0x1D,
		BONE_LEFTHANDINDEX0 = 0x1E,
		BONE_LEFTHANDINDEX1 = 0x1F,
		BONE_LEFTHANDINDEX2 = 0x20,
		BONE_LEFTHANDINDEX3 = 0x21,
		BONE_LEFTHANDINDEX4 = 0x22,
		BONE_LEFTHANDTHUMB1 = 0x23,
		BONE_LEFTHANDTHUMB2 = 0x24,
		BONE_LEFTHANDTHUMB3 = 0x25,
		BONE_LEFTHANDTHUMB4 = 0x26,
		BONE_LEFTFOREARMCLOTH = 0x27,
		BONE_LEFTELBOWROLL = 0x28,
		BONE_LEFTARMCLOTH = 0x29,
		BONE_LEFTSHOULDERPHYS1 = 0x2A,
		BONE_NECK = 0x2B,
		BONE_NECK1 = 0x2C,
		BONE_HEAD = 0x2D,
		BONE_HEADEND = 0x2E,
		BONE_FACE = 0x2F,
		BONE_OFFSET_JAW = 0x30,
		BONE_JAW = 0x31,
		BONE_OFFSET_LEFTLOWERLIP = 0x32,
		BONE_LEFTLOWERLIP = 0x33,
		BONE_OFFSET_LOWERLIP = 0x34,
		BONE_LOWERLIP = 0x35,
		BONE_OFFSET_RIGHTLOWERLIP = 0x36,
		BONE_RIGHTLOWERLIP = 0x37,
		BONE_OFFSET_TONGUE = 0x38,
		BONE_TONGUE = 0x39,
		BONE_OFFSET_TONGUETIP = 0x3A,
		BONE_TONGUETIP = 0x3B,
		BONE_OFFSET_CHIN = 0x3C,
		BONE_CHIN = 0x3D,
		BONE_OFFSET_LEFTEYE = 0x3E,
		BONE_LEFTEYE = 0x3F,
		BONE_OFFSET_RIGHTEYE = 0x40,
		BONE_RIGHTEYE = 0x41,
		BONE_OFFSET_LEFTUPCHEEK = 0x42,
		BONE_LEFTUPCHEEK = 0x43,
		BONE_OFFSET_LEFTUPINNERCHEEK = 0x44,
		BONE_LEFTUPINNERCHEEK = 0x45,
		BONE_OFFSET_RIGHTUPINNERCHEEK = 0x46,
		BONE_RIGHTUPINNERCHEEK = 0x47,
		BONE_OFFSET_RIGHTUPCHEEK = 0x48,
		BONE_RIGHTUPCHEEK = 0x49,
		BONE_OFFSET_LEFTCHEEK = 0x4A,
		BONE_LEFTCHEEK = 0x4B,
		BONE_OFFSET_RIGHTCHEEK = 0x4C,
		BONE_RIGHTCHEEK = 0x4D,
		BONE_OFFSET_LEFTMOUTH = 0x4E,
		BONE_LEFTMOUTH = 0x4F,
		BONE_OFFSET_LEFTUPPERLIP = 0x50,
		BONE_LEFTUPPERLIP = 0x51,
		BONE_OFFSET_UPPERLIP = 0x52,
		BONE_UPPERLIP = 0x53,
		BONE_OFFSET_RIGHTUPPERLIP = 0x54,
		BONE_RIGHTUPPERLIP = 0x55,
		BONE_OFFSET_RIGHTMOUTH = 0x56,
		BONE_RIGHTMOUTH = 0x57,
		BONE_OFFSET_LEFTUPEYELID = 0x58,
		BONE_LEFTUPEYELID = 0x59,
		BONE_OFFSET_RIGHTUPEYELID = 0x5A,
		BONE_RIGHTUPEYELID = 0x5B,
		BONE_OFFSET_LEFTLOWEYELID = 0x5C,
		BONE_LEFTLOWEYELID = 0x5D,
		BONE_OFFSET_RIGHTLOWEYELID = 0x5E,
		BONE_RIGHTLOWEYELID = 0x5F,
		BONE_OFFSET_LEFTINNEREYEBROW = 0x60,
		BONE_LEFTINNEREYEBROW = 0x61,
		BONE_OFFSET_LEFTOUTEREYEBROW = 0x62,
		BONE_LEFTOUTEREYEBROW = 0x63,
		BONE_OFFSET_RIGHTINNEREYEBROW = 0x64,
		BONE_RIGHTINNEREYEBROW = 0x65,
		BONE_OFFSET_RIGHTOUTEREYEBROW = 0x66,
		BONE_RIGHTOUTEREYEBROW = 0x67,
		BONE_OFFSET_LEFTNOSE = 0x68,
		BONE_LEFTNOSE = 0x69,
		BONE_OFFSET_RIGHTNOSE = 0x6A,
		BONE_RIGHTNOSE = 0x6B,
		BONE_OFFSET_LEFTCREASE = 0x6C,
		BONE_LEFTCREASE = 0x6D,
		BONE_OFFSET_RIGHTCREASE = 0x6E,
		BONE_RIGHTCREASE = 0x6F,
		BONE_OFFSET_LEFTMIDDLEEYEBROW = 0x70,
		BONE_LEFTMIDDLEEYEBROW = 0x71,
		BONE_OFFSET_RIGHTMIDDLEEYEBROW = 0x72,
		BONE_RIGHTMIDDLEEYEBROW = 0x73,
		BONE_HEADCLOTH = 0x74,
		BONE_THROAT = 0x75,
		BONE_SPINE2PHYS1 = 0x76,
		BONE_SPINE2CLOTH = 0x77,
		BONE_RIGHTSHOULDER = 0x78,
		BONE_RIGHTARM = 0x79,
		BONE_RIGHTARMROLL = 0x7A,
		BONE_RIGHTFOREARM = 0x7B,
		BONE_RIGHTFOREARMROLL = 0x7C,
		BONE_RIGHTFOREARMROLL1 = 0x7D,
		BONE_RIGHTHAND = 0x7E,
		BONE_RIGHTHANDMIDDLE0 = 0x7F,
		BONE_RIGHTHANDMIDDLE1 = 0x80,
		BONE_RIGHTHANDMIDDLE2 = 0x81,
		BONE_RIGHTHANDMIDDLE3 = 0x82,
		BONE_RIGHTHANDMIDDLE4 = 0x83,
		BONE_RIGHTHANDRING0 = 0x84,
		BONE_RIGHTHANDRING1 = 0x85,
		BONE_RIGHTHANDRING2 = 0x86,
		BONE_RIGHTHANDRING3 = 0x87,
		BONE_RIGHTHANDRING4 = 0x88,
		BONE_RIGHTHANDPINKY0 = 0x89,
		BONE_RIGHTHANDPINKY1 = 0x8A,
		BONE_RIGHTHANDPINKY2 = 0x8B,
		BONE_RIGHTHANDPINKY3 = 0x8C,
		BONE_RIGHTHANDPINKY4 = 0x8D,
		BONE_RIGHTHANDINDEX0 = 0x8E,
		BONE_RIGHTHANDINDEX1 = 0x8F,
		BONE_RIGHTHANDINDEX2 = 0x90,
		BONE_RIGHTHANDINDEX3 = 0x91,
		BONE_RIGHTHANDINDEX4 = 0x92,
		BONE_RIGHTHANDTHUMB1 = 0x93,
		BONE_RIGHTHANDTHUMB2 = 0x94,
		BONE_RIGHTHANDTHUMB3 = 0x95,
		BONE_RIGHTHANDTHUMB4 = 0x96,
		BONE_RIGHTFOREARMCLOTH = 0x97,
		BONE_RIGHTELBOWROLL = 0x98,
		BONE_RIGHTARMCLOTH = 0x99,
		BONE_RIGHTSHOULDERPHYS1 = 0x9A,
		BONE_WEP_ROOT = 0x9B,
		BONE_WEP_EXTRA1 = 0x9C,
		BONE_WEP_TRIGGER = 0x9D,
		BONE_WEP_SLIDE = 0x9E,
		BONE_WEP_GRENADE1 = 0x9F,
		BONE_WEP_GRENADE2 = 0xA0,
		BONE_WEP_MAG = 0xA1,
		BONE_WEP_MAG_AMMO = 0xA2,
		BONE_WEP_PHYSIC1 = 0xA3,
		BONE_WEP_PHYSIC2 = 0xA4,
		BONE_WEP_PHYSIC3 = 0xA5,
		BONE_WEP_BELT1 = 0xA6,
		BONE_WEP_BELT2 = 0xA7,
		BONE_WEP_BELT3 = 0xA8,
		BONE_WEP_BELT4 = 0xA9,
		BONE_WEP_BELT5 = 0xAA,
		BONE_WEP_BIPOD1 = 0xAB,
		BONE_WEP_BIPOD2 = 0xAC,
		BONE_WEP_BIPOD3 = 0xAD,
		BONE_IK_JOINT_LEFTHAND = 0xAE,
		BONE_IK_JOINT_RIGHTHAND = 0xAF,
		BONE_WEP_EXTRA2 = 0xB0,
		BONE_WEP_EXTRA3 = 0xB1,
		BONE_WEP_AIM = 0xB2,
		BONE_LEFTSPINE1PHYS1 = 0xB3,
		BONE_RIGHTSPINE1PHYS1 = 0xB4,
		BONE_LEFTUPLEG = 0xB5,
		BONE_LEFTUPLEGROLL = 0xB6,
		BONE_LEFTLEG = 0xB7,
		BONE_LEFTFOOT = 0xB8,
		BONE_LEFTTOEBASE = 0xB9,
		BONE_LEFTTOE = 0xBA,
		BONE_LEFTLEGCLOTH = 0xBB,
		BONE_LEFTKNEEROLL = 0xBC,
		BONE_LEFTHIPSROLL = 0xBD,
		BONE_LEFTUPLEGCLOTH = 0xBE,
		BONE_HIPSCENTERPHYS = 0xBF,
		BONE_HIPSCLOTH = 0xC0,
		BONE_LEFTHIPSFRONTPHYS = 0xC1,
		BONE_LEFTHIPSBACKPHYS = 0xC2,
		BONE_LEFTHIPSSIDEPHYS = 0xC3,
		BONE_RIGHTUPLEG = 0xC4,
		BONE_RIGHTUPLEGROLL = 0xC5,
		BONE_RIGHTLEG = 0xC6,
		BONE_RIGHTFOOT = 0xC7,
		BONE_RIGHTTOEBASE = 0xC8,
		BONE_RIGHTTOE = 0xC9,
		BONE_RIGHTLEGCLOTH = 0xCA,
		BONE_RIGHTKNEEROLL = 0xCB,
		BONE_RIGHTHIPSROLL = 0xCC,
		BONE_RIGHTUPLEGCLOTH = 0xCD,
		BONE_RIGHTHIPSFRONTPHYS = 0xCE,
		BONE_RIGHTHIPSBACKPHYS = 0xCF,
		BONE_RIGHTHIPSSIDEPHYS = 0xD0,
		enumTypeEnd
	};

	extern bool AutoFire;

	static fb::ClientPlayer* GetLocalPlayer()
	{
		fb::ClientGameContext* pGameContext = fb::ClientGameContext::Singleton();

		if( !pGameContext )
			return NULL;

		fb::ClientPlayerManager* pPlayerManager = pGameContext->m_clientPlayerManager;

		if( !pPlayerManager )
			return NULL;

		if( !pPlayerManager->m_localPlayer )
			return NULL;

		return pPlayerManager->m_localPlayer;
	}
	static bool ScreenProject( fb::Vec3* vPos, fb::Vec3* vOut )
	{
		if( !vPos || !vOut )
			return false;

		fb::DxRenderer* dxRenderer = fb::DxRenderer::Singleton();

		if( !dxRenderer )
			return false;

		fb::GameRenderer* gameRenderer = fb::GameRenderer::Singleton();

		if( !gameRenderer )
			return false;

		if( !gameRenderer->m_viewParams.view.Update())
			return false;

		float mX = ( ( float )dxRenderer->m_screenInfo.m_nWidth * 0.5f );
		float mY = ( ( float )dxRenderer->m_screenInfo.m_nHeight * 0.5f );

		fb::Mat4 screenTransform = gameRenderer->m_viewParams.view.m_viewProjectionMatrix;
		fb::Vec3 origin = ( *vPos );

		float w = screenTransform[0][3] * origin.x + screenTransform[1][3] * origin.y + screenTransform[2][3] * origin.z + screenTransform[3][3];

		if( w < 0.0001f )
		{
			vOut->z = w;

			return false;
		}
		float x = screenTransform[0][0] * origin.x + screenTransform[1][0] * origin.y + screenTransform[2][0] * origin.z + screenTransform[3][0];
		float y = screenTransform[0][1] * origin.x + screenTransform[1][1] * origin.y + screenTransform[2][1] * origin.z + screenTransform[3][1];

		vOut->x = mX + mX * x / w;
		vOut->y = mY - mY * y / w;
		vOut->z = w;

		return true;
	}
	static float GetDistance( fb::Vec3 v1, fb::Vec3 v2 )
	{
		return v1.DistanceFrom( v2 );
	}

	static float GetDistance( fb::ClientPlayer* pLocal, fb::ClientPlayer* pPlayer )
	{
		/*if (!IsValid(pLocal) || !IsValid(pPlayer))
			return 0.0f;

		fb::ClientSoldierEntity* plEnt = pLocal->GetSoldier();

		if (!IsValid(plEnt))
			return 0.0f;

		fb::ClientSoldierEntity* pEnt = pPlayer->GetSoldier();

		if (!IsValid(pEnt))
			return 0.0f;

		return ( plEnt->m_replicatedController->m_state.position.DistanceFrom( pEnt->m_replicatedController->m_state.position ) );*/
		return 0.0f;
	}

	static float Distance2D(float x1,float y1,float x2,float y2)
	{

		return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	}

	static float Distance3D(float x1,float y1,float z1,float x2,float y2,float z2)
	{

		return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
	}

	inline float XAngle(float x1,float y1,float x2,float y2,float myangle)
	{
		float dl=Distance2D(x1,y1,x2,y2);
		if(dl==0)dl=1.0;
		float dl2=abs(x2-x1);
		float teta=((180.0/M_PI)*acos(dl2/dl));
		if(x2<x1)teta=180-teta;
		if(y2<y1)teta=teta*-1.0 ;
		teta=teta-myangle;
		if(teta>180.0)teta=(360.0-teta)*(-1.0);
		if(teta<-180.0)teta=(360.0+teta);
		return teta;
	}

	inline float  YAngle(float x1,float y1,float z1,float x2,float y2,float z2,float myangle)
	{

		float dl=Distance3D(x1,y1,z1,x2,y2,z2);
		if(dl==0)dl=1;
		float dl2=abs(z2-z1);
		float teta=((180.0/M_PI)*asin(dl2/dl));;
		if(z2<z1)teta=teta*-1.0 ;
		teta=myangle+teta;
		if(teta>180.0)teta=(360.0-teta)*(-1.0);
		if(teta<-180.0)teta=(360.0+teta);
		return (-1)*teta;
	}

	static bool IsValid( void* ptr )
	{
		if( ptr && HIWORD( ptr ) != NULL )
			return true;

		return false;
	}

	static bool IsValidPointer(void* ptr)
	{
		if (ptr && HIWORD(ptr) != NULL)
			return true;

		return false;
	}

	static bool IsEnemy( fb::ClientPlayer* pPlayer )
	{
		fb::ClientPlayer* pLocal = fb::GetLocalPlayer();

		if( IsValid( pLocal ) && IsValid( pPlayer ) )
		{
			if( pLocal->m_teamId != pPlayer->m_teamId )
				return true;
		}

		return false;
	}
	static void uhRotatePointAlpha(float *outV, float x, float y, float z, float cx, float cy, float cz, float alpha)
	{
		D3DXMATRIX rot1;
		D3DXVECTOR4 vec;
		vec.x=x-cx;
		vec.z=y-cy;
		vec.y=z-cz;
		vec.w=1.0;
		D3DXMatrixRotationY(&rot1,alpha*D3DX_PI/180.0);
		D3DXVec4Transform(&vec,&vec,&rot1);
		outV[0]=vec.x+cx;
		outV[1]=vec.z+cy;
		outV[2]=vec.y+cz;
	};

	/*BOOL TraceBones(ClientSoldierEntity *pSoldier, Vec3* vecout, int bone) // Fragger
	{
		if (!pSoldier)
			return FALSE;
		fb::ClientRagdollComponent *pCRC = pSoldier->getComponent<fb::ClientRagdollComponent>("ClientRagdollComponent");
		if (!VALIDPOINTER(pCRC)) return FALSE;
		ant::UpdatePoseResultData pUPRD = pCRC->m_updatePoseResultData;
		if (!VALIDPOINTER(pUPRD.m_activeWorldTransforms)) return FALSE;
		memcpy_s(vecout, sizeof(Vec3), &(pUPRD.m_activeWorldTransforms[bone].transAndScale), sizeof(Vec3));
		return TRUE;
	}*/

	static bool GetBonePos( fb::ClientSoldierEntity* pEnt, int iBone, fb::Vec3* vOut )
	{
		if (!IsValid(pEnt))
			return false;

		if( !iBone )
			return false;

		if (!IsValid(vOut))
			return false;

		fb::ClientAntAnimatableComponent* pAnt = pEnt->m_animatableComponent[1];

		if (!IsValid(pAnt))
			return false;

		pAnt->m_handler.m_hadVisualUpdate = true; // Crash here

		fb::Animatable* pAnimatable = pAnt->m_handler.m_animatable;

		if (!IsValid(pAnimatable))
			return false;

		if( pAnimatable->m_updatePoseResultData.m_validTransforms )
		{
			ant::QuatTransform* pQuat = pAnimatable->m_updatePoseResultData.m_activeWorldTransforms;

			if (!IsValid(pQuat))
				return false;

			fb::Vec3 vTmp = pQuat[iBone].transAndScale;

			vOut->x = vTmp.x;
			vOut->y = vTmp.y;
			vOut->z = vTmp.z;

			return true;
		}

		return false;
	}

	static fb::ClientSoldierWeapon2* getSoldiereWeapon(fb::ClientSoldierEntity *Soldier)
	{

		if(Soldier->m_soldierWeaponsComponent != NULL && Soldier->m_soldierWeaponsComponent->m_currentAnimatedWeaponHandler != NULL && Soldier->m_soldierWeaponsComponent->m_currentAnimatedWeaponHandler->m_currentAnimatedWeapon != NULL)
		{
			return Soldier->m_soldierWeaponsComponent->m_currentAnimatedWeaponHandler->m_currentAnimatedWeapon;
		} else
		{
			return NULL;
		}
	}

	static fb::ShotConfigData* GetShotConfigData( fb::ClientSoldierEntity* pEnt )
	{
		if( !IsValid( pEnt ) )
			return NULL;

		fb::ClientSoldierWeapon2* pWeapon = getSoldiereWeapon(pEnt);

		if( !IsValid( pWeapon ) )
			return NULL;

		fb::WeaponFiring* pFiring = pWeapon->m_predictedFiring;

		if( !IsValid( pFiring ) )
			return NULL;

		fb::WeaponFiringData* pFiringData = pFiring->m_data;

		if( !IsValid( pFiringData ) )
			return NULL;

		fb::FiringFunctionData* pFiringFunc = pFiringData->m_primaryFire.m_ptr;

		if( !IsValid( pFiringFunc ) )
			return NULL;

		fb::ShotConfigData* pShot = &pFiringFunc->m_shot;

		if( !IsValid( pShot ) )
			return NULL;

		return pShot;
	}

	static fb::Deviation* GetRecoil( fb::GunSway* pGunSway )
	{
		if( !IsValid( pGunSway ) )
			return NULL;

		return &pGunSway->m_currentRecoilDeviation;
	}

	static fb::Deviation* GetSpread( fb::GunSway* pGunSway )
	{
		if( !IsValid( pGunSway ) )
			return NULL;

		fb::WeaponFiringUpdateContext m_context;

		unsigned int m_oldSeed = pGunSway->m_random.m_value;
		pGunSway->m_random.m_nextNormalIsValid = 0;

		if( IsValid( fb::ClientGameContext::Singleton()->m_gameTime ) )

			m_context.ticks = fb::ClientGameContext::Singleton()->m_gameTime->m_ticks;
		pGunSway->primaryFireShotSpawnedCallback( 0.0f, true, &m_context );

		pGunSway->m_random.m_value = m_oldSeed;

		return &pGunSway->m_currentDispersionDeviation;
	}

	static fb::GunSway* ToGunSway( fb::WeaponSway* pWeaponSway )
	{
		if( !IsValid( pWeaponSway ) )
			return NULL;

		return reinterpret_cast< fb::GunSway* >( pWeaponSway );
	}

	static fb::WeaponSway* GetWeaponSway( fb::ClientSoldierEntity* pEnt )
	{
		if( !IsValid( pEnt ) )
			return NULL;

		fb::ClientSoldierWeapon2* pWeapon = getSoldiereWeapon(pEnt);

		if( !IsValid( pWeapon ) )
			return NULL;

		fb::WeaponFiring* pFiring = pWeapon->m_predictedFiring;

		if( !IsValid( pFiring ) )
			return NULL;

		return pFiring->m_weaponSway;
	}

	static bool isVisible(fb::Vec3* vFrom, fb::Vec3* vTo)
	{
		fb::ClientGameContext* gameContext = fb::ClientGameContext::Singleton();
		if(!gameContext || !gameContext->m_level || !gameContext->m_level->m_PhysicsManager)
			return false;

		fb::GameWorld* pWorld = gameContext->m_level->m_gameWorld;
		if( !IsValid( pWorld ) )
			return false;

		fb::physics::IPhysicsRayCaster* rayCaster = &pWorld->m_rayCaster;

		fb::physics::RayCastHit ray;

		// need D3DXVECTOR4
		__declspec( align( 16 ) )fb::Vec3 vFinalFrom = *vFrom;
		__declspec( align( 16 ) )fb::Vec3 vFinalTo = *vTo; // TNX

		bool ret = !rayCaster->physicsRayQuery("OnGroundState::update", vFinalFrom, vFinalTo, ray,fb::physics::DontCheckWater | fb::physics::DontCheckRagdoll | fb::physics::DontCheckCharacter | fb::physics::DontCheckPhantoms, NULL );

		return ret;
	}

	static bool RayTrace(fb::Vec3* vFrom, fb::Vec3* vTo)
	{
		fb::ClientGameContext* gameContext = fb::ClientGameContext::Singleton();
		if (!gameContext || !gameContext->m_level || !gameContext->m_level->m_PhysicsManager)
			return false;

		fb::GameWorld* pWorld = gameContext->m_level->m_gameWorld;
		if (!IsValid(pWorld))
			return false;

		fb::physics::IPhysicsRayCaster* rayCaster = &pWorld->m_rayCaster;

		fb::physics::RayCastHit ray;

		// need D3DXVECTOR4
		__declspec(align(16))fb::Vec3 vFinalFrom = *vFrom;
		__declspec(align(16))fb::Vec3 vFinalTo = *vTo; // TNX

		bool ret = !rayCaster->physicsRayQuery("OnGroundState::update", vFinalFrom, vFinalTo, ray,
			// CheckDetailMesh |
			// IsAsyncRaycast |
			fb::physics::DontCheckWater |
			// DontCheckTerrain |
			fb::physics::DontCheckRagdoll |
			fb::physics::DontCheckCharacter |
			// DontCheckGroup |
			fb::physics::DontCheckPhantoms,
			/*fb::physics::DontCheckWater | fb::physics::DontCheckRagdoll | fb::physics::DontCheckCharacter | fb::physics::DontCheckPhantoms*/
			NULL);

		return ret;
	}

	static bool rIsVisible(Vec3* target) // Fragger
	{
		//if (!GetLocalPlayer->physics() ||
			//!pMySoldier->physics()->m_manager)
			//return false;

		ClientGameContext* gameContext = fb::ClientGameContext::Singleton();
		if (!gameContext || !gameContext->m_level || !gameContext->m_level->m_PhysicsManager)
			return false;

		GameWorld* pWorld = gameContext->m_level->m_gameWorld;
		if (!IsValid(pWorld))
			return false;

		physics::IPhysicsRayCaster* pIRC = &pWorld->m_rayCaster;
		if (!IsValid(pIRC))
			return false;
		physics::RayCastHit pRCH;
		__declspec(align(16)) Vec3 Me;
		fb::GetBonePos(gameContext->m_clientPlayerManager->m_localPlayer->m_soldier.GetData(), fb::BONE_HEADEND, &Me);
		__declspec(align(16)) Vec3 enemy = *target;
		/*enum RayCastFlags
		{
			CheckDetailMesh = 0x1,
			IsAsyncRaycast = 0x2,
			DontCheckWater = 0x4,
			DontCheckTerrain = 0x8,
			DontCheckRagdoll = 0x10,
			DontCheckCharacter = 0x20,
			DontCheckGroup = 0x40,
			DontCheckPhantoms = 0x80,
		};*/
		return !pIRC->physicsRayQuery("x", Me, enemy, pRCH, 0x4 | 0x10 | 0x20 | 0x80/*fb::physics::DontCheckWater | fb::physics::DontCheckRagdoll | fb::physics::DontCheckCharacter | fb::physics::DontCheckGroup | fb::physics::DontCheckPhantoms*/, NULL);
	}

	static void Flashlight( fb::ClientSoldierEntity* pEnt, bool state )
	{
		if( !IsValid( pEnt ) )
			return;

		fb::ClientSoldierWeaponsComponent* pSoldierWeapon = pEnt->m_soldierWeaponsComponent;

		if( !IsValid( pSoldierWeapon ) )
			return;

		pSoldierWeapon->m_weaponLightDisabled = !state;
	}

	static bool worldToScreen(Vector From, Vector& To)
	{
		//if (From == NULL || To == NULL)
		//	return false;

		fb::DxRenderer* dxRenderer = fb::DxRenderer::Singleton();

		if (!IsValid(dxRenderer))
			return false;

		fb::GameRenderer* renderer = fb::GameRenderer::Singleton();

		if (!IsValid(renderer))
			return false;

		renderer->m_viewParams.view.m_viewProjectionMatrix = fb::Mat4();

		if (renderer->m_viewParams.view.Update() == false)
			return false;

		float CX = static_cast< float >(dxRenderer->m_screenInfo.m_nWindowWidth) * 0.5f;
		float CY = static_cast< float >(dxRenderer->m_screenInfo.m_nWindowHeight) * 0.5f;

		fb::Mat4 ScreenTransform = renderer->m_viewParams.view.m_viewProjectionMatrix;

		Vector Origin = (From);

		float W =
			ScreenTransform.m_rows[0].w * Origin.x +
			ScreenTransform.m_rows[1].w * Origin.y +
			ScreenTransform.m_rows[2].w * Origin.z +
			ScreenTransform.m_rows[3].w;

		if (W < 0.0001f)
		{
			To.z = W;

			return false;
		}

		float X =
			ScreenTransform.m_rows[0].x * Origin.x +
			ScreenTransform.m_rows[1].x * Origin.y +
			ScreenTransform.m_rows[2].x * Origin.z +
			ScreenTransform.m_rows[3].x;

		float Y =
			ScreenTransform.m_rows[0].y * Origin.x +
			ScreenTransform.m_rows[1].y * Origin.y +
			ScreenTransform.m_rows[2].y * Origin.z +
			ScreenTransform.m_rows[3].y;

		To.x = CX + CX * X / W;
		To.y = CY - CY * Y / W;
		To.z = W;

		return true;
	}
};
