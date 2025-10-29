# SmartCar STM32 Project 🚗

Système embarqué de supervision et de contrôle intelligent pour mini-véhicule électrique basé sur STM32F407VG.

## 📋 Description

Ce projet implémente un système de contrôle moteur intelligent avec surveillance de température pour assurer la sécurité du conducteur. Le système contrôle les mouvements du véhicule (rotation lente vers la droite) et arrête automatiquement le moteur si la température atteint un niveau dangereux.

## 📁 Structure du Projet

```
SmartCar_STM32_Project/
│
├── Iar_stm32_project/          # Projet principal IAR Embedded Workbench
│   ├── Inc/                    # Fichiers d'en-tête (.h)
│   │   ├── main.h
│   │   ├── motor_control.h     # Contrôle moteur avec sécurité température
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   │
│   ├── Src/                    # Fichiers sources (.c)
│   │   ├── main.c
│   │   ├── motor_control.c     # Implémentation contrôle moteur
│   │   ├── stm32f4xx_hal_msp.c
│   │   ├── stm32f4xx_it.c
│   │   └── system_stm32f4xx.c
│   │
│   ├── EWARM/                  # Configuration IAR EWARM
│   │   ├── Project.eww         # Workspace IAR
│   │   ├── Project.ewp         # Fichier projet
│   │   ├── startup_stm32f407xx.s
│   │   └── settings/
│   │
│   ├── MDK-ARM/                # Configuration Keil MDK-ARM
│   │   ├── Project.uvprojx
│   │   └── startup_stm32f407xx.s
│   │
│   └── SW4STM32/               # Configuration System Workbench for STM32
│       └── STM32F4-Discovery/
│
├── data/                       # Données et ressources
├── settings/                   # Paramètres de l'environnement
└── README.md                   # Ce fichier
```

## 🎯 Fonctionnalités

### Contrôle Moteur Intelligent
- **Mouvement directionnel** : Rotation lente vers la droite avec vitesse différentielle
  - Moteur gauche : 21% de vitesse (pour le virage)
  - Moteur droit : 9% de vitesse (pour le virage)
  
### Système de Sécurité Thermique
- **Surveillance continue** de la température du moteur
- **Seuils de température** :
  - ⚠️ Avertissement : 75°C
  - 🚨 Critique : 85°C
- **Arrêt d'urgence automatique** si température ≥ 85°C
- **Verrouillage de sécurité** : empêche le redémarrage tant que la température reste élevée

## 🔧 API Motor Control

### Fonctions principales

```c
void Motor_Init(void);                          // Initialisation du système
void Motor_MoveRightSlowly(void);              // Tourner à droite lentement
void Motor_Stop(void);                          // Arrêt normal
void Motor_EmergencyStop(void);                // Arrêt d'urgence
void Motor_UpdateTemperature(float temp);      // Mise à jour température
void Motor_CheckTemperatureAndControl(void);   // Contrôle basé sur température
MotorState_t Motor_GetState(void);             // État actuel du moteur
MotorTempStatus_t Motor_GetTemperatureStatus(); // État température
float Motor_GetCurrentTemperature(void);       // Température actuelle
```

### États du Moteur
- `MOTOR_STATE_STOPPED` : Moteur arrêté
- `MOTOR_STATE_RUNNING` : Moteur en fonctionnement
- `MOTOR_STATE_EMERGENCY_STOP` : Arrêt d'urgence actif

### États de Température
- `MOTOR_TEMP_NORMAL` : Température normale (< 75°C)
- `MOTOR_TEMP_WARNING` : Température élevée (75-85°C)
- `MOTOR_TEMP_CRITICAL` : Température critique (≥ 85°C)

## 🚀 Utilisation

```c
#include "motor_control.h"

int main(void) {
    // Initialisation du système
    HAL_Init();
    SystemClock_Config();
    Motor_Init();
    
    while (1) {
        // Mise à jour de la température depuis le capteur
        float sensorTemp = ReadTemperatureSensor();
        Motor_UpdateTemperature(sensorTemp);
        
        // Vérification et contrôle automatique
        Motor_CheckTemperatureAndControl();
        
        // Commande de mouvement (si température OK)
        Motor_MoveRightSlowly();
        
        HAL_Delay(100);
    }
}
```

## 🛠️ Configuration Matérielle

**Microcontrôleur** : STM32F407VG
**Carte** : STM32F4-Discovery

### Configuration requise
- PWM Timer pour contrôle de vitesse moteurs
- GPIO pour direction moteurs
- ADC ou capteur I2C/SPI pour température
- LED d'avertissement (optionnel)

## 📝 Notes de Développement

- Les fichiers TODO dans `motor_control.c` indiquent les configurations hardware spécifiques à implémenter
- Adapter les pins GPIO et timers selon votre configuration matérielle
- Le système utilise HAL (Hardware Abstraction Layer) de STM32

## 📄 Licence

Voir `Iar_stm32_project/LICENSE.txt`

## 👥 Auteur

Projet SmartCar STM32 - Système embarqué intelligent
