#ifndef OPENMW_COMPONENTS_SETTINGS_CATEGORIES_GAME_H
#define OPENMW_COMPONENTS_SETTINGS_CATEGORIES_GAME_H

#include <components/detournavigator/collisionshapetype.hpp>
#include <components/settings/sanitizerimpl.hpp>
#include <components/settings/settingvalue.hpp>

#include <osg/Math>
#include <osg/Vec2f>
#include <osg/Vec3f>

#include <cstdint>
#include <string>
#include <string_view>

namespace Settings
{
    struct GameCategory : WithIndex
    {
        using WithIndex::WithIndex;

        SettingValue<int> mShowOwned{ mIndex, "Game", "show owned", makeEnumSanitizerInt({ 0, 1, 2, 3 }) };
        SettingValue<bool> mShowProjectileDamage{ mIndex, "Game", "show projectile damage" };
        SettingValue<bool> mShowMeleeInfo{ mIndex, "Game", "show melee info" };
        SettingValue<bool> mShowEnchantChance{ mIndex, "Game", "show enchant chance" };
        SettingValue<bool> mBestAttack{ mIndex, "Game", "best attack" };
        SettingValue<int> mDifficulty{ mIndex, "Game", "difficulty", makeClampSanitizerInt(-500, 500) };
        // We have to cap it since using high values (larger than 7168) will make some quests harder or impossible to
        // complete (bug #1876)
        SettingValue<int> mActorsProcessingRange{ mIndex, "Game", "actors processing range",
            makeClampSanitizerInt(3584, 7168) };
        SettingValue<bool> mClassicReflectedAbsorbSpellsBehavior{ mIndex, "Game",
            "classic reflected absorb spells behavior" };
        SettingValue<bool> mClassicCalmSpellsBehavior{ mIndex, "Game", "classic calm spells behavior" };
        SettingValue<bool> mShowEffectDuration{ mIndex, "Game", "show effect duration" };
        SettingValue<bool> mPreventMerchantEquipping{ mIndex, "Game", "prevent merchant equipping" };
        SettingValue<bool> mEnchantedWeaponsAreMagical{ mIndex, "Game", "enchanted weapons are magical" };
        SettingValue<bool> mFollowersAttackOnSight{ mIndex, "Game", "followers attack on sight" };
        SettingValue<bool> mCanLootDuringDeathAnimation{ mIndex, "Game", "can loot during death animation" };
        SettingValue<bool> mRebalanceSoulGemValues{ mIndex, "Game", "rebalance soul gem values" };
        SettingValue<bool> mUseAdditionalAnimSources{ mIndex, "Game", "use additional anim sources" };
        SettingValue<bool> mSmoothAnimTransitions{ mIndex, "Game", "smooth animation transitions" };
        SettingValue<bool> mBarterDispositionChangeIsPermanent{ mIndex, "Game",
            "barter disposition change is permanent" };
        SettingValue<int> mStrengthInfluencesHandToHand{ mIndex, "Game", "strength influences hand to hand",
            makeEnumSanitizerInt({ 0, 1, 2 }) };
        SettingValue<bool> mWeaponSheathing{ mIndex, "Game", "weapon sheathing" };
        SettingValue<bool> mShieldSheathing{ mIndex, "Game", "shield sheathing" };
        SettingValue<bool> mOnlyAppropriateAmmunitionBypassesResistance{ mIndex, "Game",
            "only appropriate ammunition bypasses resistance" };
        SettingValue<bool> mUseMagicItemAnimations{ mIndex, "Game", "use magic item animations" };
        SettingValue<bool> mNormaliseRaceSpeed{ mIndex, "Game", "normalise race speed" };
        SettingValue<float> mProjectilesEnchantMultiplier{ mIndex, "Game", "projectiles enchant multiplier",
            makeClampSanitizerFloat(0, 1) };
        SettingValue<bool> mUncappedDamageFatigue{ mIndex, "Game", "uncapped damage fatigue" };
        SettingValue<bool> mTurnToMovementDirection{ mIndex, "Game", "turn to movement direction" };
        SettingValue<bool> mSmoothMovement{ mIndex, "Game", "smooth movement" };
        SettingValue<float> mSmoothMovementPlayerTurningDelay{ mIndex, "Game", "smooth movement player turning delay",
            makeMaxSanitizerFloat(0.01f) };
        SettingValue<bool> mNPCsAvoidCollisions{ mIndex, "Game", "NPCs avoid collisions" };
        SettingValue<bool> mNPCsGiveWay{ mIndex, "Game", "NPCs give way" };
        SettingValue<bool> mSwimUpwardCorrection{ mIndex, "Game", "swim upward correction" };
        SettingValue<float> mSwimUpwardCoef{ mIndex, "Game", "swim upward coef", makeClampSanitizerFloat(-1, 1) };
        SettingValue<bool> mTrainersTrainingSkillsBasedOnBaseSkill{ mIndex, "Game",
            "trainers training skills based on base skill" };
        SettingValue<bool> mAlwaysAllowStealingFromKnockedOutActors{ mIndex, "Game",
            "always allow stealing from knocked out actors" };
        SettingValue<bool> mGraphicHerbalism{ mIndex, "Game", "graphic herbalism" };
        SettingValue<bool> mAllowActorsToFollowOverWaterSurface{ mIndex, "Game",
            "allow actors to follow over water surface" };
        SettingValue<osg::Vec3f> mDefaultActorPathfindHalfExtents{ mIndex, "Game",
            "default actor pathfind half extents", makeMaxStrictSanitizerVec3f(osg::Vec3f(0, 0, 0)) };
        SettingValue<bool> mDayNightSwitches{ mIndex, "Game", "day night switches" };
        SettingValue<DetourNavigator::CollisionShapeType> mActorCollisionShapeType{ mIndex, "Game",
            "actor collision shape type" };
        SettingValue<bool> mPlayerMovementIgnoresAnimation{ mIndex, "Game", "player movement ignores animation" };
        SettingValue<bool> mBrutalAdrenalineHud{ mIndex, "Game", "brutal adrenaline hud" };
        SettingValue<bool> mBrutalEnabled{ mIndex, "Game", "brutal enabled" };
        SettingValue<bool> mBrutalFirstPersonBody{ mIndex, "Game", "brutal first person body" };
        SettingValue<bool> mBrutalRagdoll{ mIndex, "Game", "brutal ragdoll" };
        SettingValue<bool> mBrutalDynamicProps{ mIndex, "Game", "brutal dynamic props" };
        SettingValue<bool> mBrutalEquipmentPhysics{ mIndex, "Game", "brutal equipment physics" };
        SettingValue<bool> mBrutalMeleeGeometry{ mIndex, "Game", "brutal melee geometry" };
        SettingValue<bool> mBrutalWallBounce{ mIndex, "Game", "brutal wall bounce" };
        SettingValue<bool> mBrutalStamina{ mIndex, "Game", "brutal stamina" };
        SettingValue<bool> mBrutalSlide{ mIndex, "Game", "brutal slide" };
        SettingValue<bool> mBrutalControlsEnabled{ mIndex, "Game", "brutal controls enabled" };
        SettingValue<bool> mBrutalFastAttacks{ mIndex, "Game", "brutal fast attacks" };
        SettingValue<bool> mBrutalClinch{ mIndex, "Game", "brutal clinch" };
        SettingValue<bool> mBrutalCombatSounds{ mIndex, "Game", "brutal combat sounds" };
        SettingValue<bool> mBrutalBloodEffects{ mIndex, "Game", "brutal blood effects" };
        SettingValue<bool> mBrutalSparkEffects{ mIndex, "Game", "brutal spark effects" };
        SettingValue<bool> mBrutalImpactEffects{ mIndex, "Game", "brutal impact effects" };
        SettingValue<bool> mKineticPlayerMovement{ mIndex, "Game", "kinetic player movement" };
        SettingValue<bool> mDarkMessiahNpcCombat{ mIndex, "Game", "dark messiah npc combat" };
        SettingValue<bool> mDarkMessiahSwordParry{ mIndex, "Game", "dark messiah sword parry" };
        SettingValue<float> mDarkMessiahParryWindow{ mIndex, "Game", "dark messiah parry window", makeClampSanitizerFloat(0, 1) };
        SettingValue<bool> mDarkMessiahComboAttacks{ mIndex, "Game", "dark messiah combo attacks" };
        SettingValue<bool> mDarkMessiahStandUp{ mIndex, "Game", "dark messiah stand up" };
        SettingValue<bool> mBrutalClassicFirstPerson{ mIndex, "Game", "brutal classic first person" };
        SettingValue<bool> mBrutalClassicNpcAnimations{ mIndex, "Game", "brutal classic npc animations" };
        SettingValue<bool> mBrutalDismemberment{ mIndex, "Game", "brutal dismemberment" };
        SettingValue<bool> mBrutalKickAlwaysRagdoll{ mIndex, "Game", "brutal kick always ragdoll" };
        SettingValue<bool> mMeleeContactAlwaysHits{ mIndex, "Game", "melee contact always hits" };
        SettingValue<bool> mNpcBloodDecals{ mIndex, "Game", "npc blood decals" };
        SettingValue<float> mNpcHurtHealthFraction{ mIndex, "Game", "npc hurt health fraction",
            makeClampSanitizerFloat(0, 1) };
        SettingValue<bool> mPlayerHitRecovery{ mIndex, "Game", "player hit recovery" };
        SettingValue<float> mPlayerFatigueRegenFraction{ mIndex, "Game", "player fatigue regen fraction",
            makeClampSanitizerFloat(0, 10) };
        SettingValue<bool> mPlayerActiveShieldBlock{ mIndex, "Game", "player active shield block" };
        SettingValue<bool> mRagdollEnhancement{ mIndex, "Game", "ragdoll enhancement" };
        SettingValue<bool> mRagdollSoftStops{ mIndex, "Game", "ragdoll soft stops" };
        SettingValue<bool> mRagdollNeutralPose{ mIndex, "Game", "ragdoll neutral pose" };
        SettingValue<bool> mRagdollLimbResponse{ mIndex, "Game", "ragdoll limb response" };
        SettingValue<bool> mRagdollSelfCollision{ mIndex, "Game", "ragdoll self collision" };
        SettingValue<float> mRagdollSoftStrength{ mIndex, "Game", "ragdoll soft strength", makeClampSanitizerFloat(0, 3) };
        SettingValue<float> mRagdollToneStrength{ mIndex, "Game", "ragdoll tone strength", makeClampSanitizerFloat(0, 3) };
        SettingValue<float> mRagdollImpactStrength{ mIndex, "Game", "ragdoll impact strength", makeClampSanitizerFloat(0, 3) };
        SettingValue<bool> mFirstPersonRigidViewmodel{ mIndex, "Game", "first person rigid viewmodel" };
        SettingValue<bool> mFirstPersonFullBody{ mIndex, "Game", "first person full body" };
        SettingValue<bool> mFirstPersonBodyAllowThirdPerson{ mIndex, "Game", "first person body allow third person" };
        SettingValue<float> mFirstPersonBodyForward{ mIndex, "Game", "first person body forward", makeClampSanitizerFloat(0, 40) };
        SettingValue<bool> mTerrainObstructsFocus{ mIndex, "Game", "terrain obstructs focus" };
    };
}

#endif
