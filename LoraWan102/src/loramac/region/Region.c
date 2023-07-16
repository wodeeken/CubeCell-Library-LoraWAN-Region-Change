/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC region implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ) and Daniel Jaeckle ( STACKFORCE )
*/
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "../LoRaMac.h"
#if defined(__asr650x__)
#include "timeServer.h"
#else
#include "timer.h"
#endif
#include "RegionEU868.h"

// Regional includes
#include "Region.h"






#include "RegionEU868.h"
#define EU868_CASE                                 case LORAMAC_REGION_EU868:
#define EU868_IS_ACTIVE( )                         EU868_CASE { return true; }
#define EU868_GET_PHY_PARAM( )                     EU868_CASE { return RegionEU868GetPhyParam( getPhy ); }
#define EU868_SET_BAND_TX_DONE( )                  EU868_CASE { RegionEU868SetBandTxDone( txDone ); break; }
#define EU868_INIT_DEFAULTS( )                     EU868_CASE { RegionEU868InitDefaults( type ); break; }
#define EU868_VERIFY( )                            EU868_CASE { return RegionEU868Verify( verify, phyAttribute ); }
#define EU868_APPLY_CF_LIST( )                     EU868_CASE { RegionEU868ApplyCFList( applyCFList ); break; }
#define EU868_CHAN_MASK_SET( )                     EU868_CASE { return RegionEU868ChanMaskSet( chanMaskSet ); }
#define EU868_ADR_NEXT( )                          EU868_CASE { return RegionEU868AdrNext( adrNext, drOut, txPowOut, adrAckCounter ); }
#define EU868_COMPUTE_RX_WINDOW_PARAMETERS( )      EU868_CASE { RegionEU868ComputeRxWindowParameters( datarate, minRxSymbols, rxError, rxConfigParams ); break; }
#define EU868_RX_CONFIG( )                         EU868_CASE { return RegionEU868RxConfig( rxConfig, datarate ); }
#define EU868_TX_CONFIG( )                         EU868_CASE { return RegionEU868TxConfig( txConfig, txPower, txTimeOnAir ); }
#define EU868_LINK_ADR_REQ( )                      EU868_CASE { return RegionEU868LinkAdrReq( linkAdrReq, drOut, txPowOut, nbRepOut, nbBytesParsed ); }
#define EU868_RX_PARAM_SETUP_REQ( )                EU868_CASE { return RegionEU868RxParamSetupReq( rxParamSetupReq ); }
#define EU868_NEW_CHANNEL_REQ( )                   EU868_CASE { return RegionEU868NewChannelReq( newChannelReq ); }
#define EU868_TX_PARAM_SETUP_REQ( )                EU868_CASE { return RegionEU868TxParamSetupReq( txParamSetupReq ); }
#define EU868_DL_CHANNEL_REQ( )                    EU868_CASE { return RegionEU868DlChannelReq( dlChannelReq ); }
#define EU868_ALTERNATE_DR( )                      EU868_CASE { return RegionEU868AlternateDr( alternateDr ); }
#define EU868_CALC_BACKOFF( )                      EU868_CASE { RegionEU868CalcBackOff( calcBackOff ); break; }
#define EU868_NEXT_CHANNEL( )                      EU868_CASE { return RegionEU868NextChannel( nextChanParams, channel, time, aggregatedTimeOff ); }
#define EU868_CHANNEL_ADD( )                       EU868_CASE { return RegionEU868ChannelAdd( channelAdd ); }
#define EU868_CHANNEL_REMOVE( )                    EU868_CASE { return RegionEU868ChannelsRemove( channelRemove ); }
#define EU868_SET_CONTINUOUS_WAVE( )               EU868_CASE { RegionEU868SetContinuousWave( continuousWave ); break; }
#define EU868_APPLY_DR_OFFSET( )                   EU868_CASE { return RegionEU868ApplyDrOffset( downlinkDwellTime, dr, drOffset ); }
#define EU868_RX_BEACON_SETUP( )                   EU868_CASE { RegionEU868RxBeaconSetup( rxBeaconSetup, outDr ); }





#include "RegionUS915.h"
#define US915_CASE                                 case LORAMAC_REGION_US915:
#define US915_IS_ACTIVE( )                         US915_CASE { return true; }
#define US915_GET_PHY_PARAM( )                     US915_CASE { return RegionUS915GetPhyParam( getPhy ); }
#define US915_SET_BAND_TX_DONE( )                  US915_CASE { RegionUS915SetBandTxDone( txDone ); break; }
#define US915_INIT_DEFAULTS( )                     US915_CASE { RegionUS915InitDefaults( type ); break; }
#define US915_VERIFY( )                            US915_CASE { return RegionUS915Verify( verify, phyAttribute ); }
#define US915_APPLY_CF_LIST( )                     US915_CASE { RegionUS915ApplyCFList( applyCFList ); break; }
#define US915_CHAN_MASK_SET( )                     US915_CASE { return RegionUS915ChanMaskSet( chanMaskSet ); }
#define US915_ADR_NEXT( )                          US915_CASE { return RegionUS915AdrNext( adrNext, drOut, txPowOut, adrAckCounter ); }
#define US915_COMPUTE_RX_WINDOW_PARAMETERS( )      US915_CASE { RegionUS915ComputeRxWindowParameters( datarate, minRxSymbols, rxError, rxConfigParams ); break; }
#define US915_RX_CONFIG( )                         US915_CASE { return RegionUS915RxConfig( rxConfig, datarate ); }
#define US915_TX_CONFIG( )                         US915_CASE { return RegionUS915TxConfig( txConfig, txPower, txTimeOnAir ); }
#define US915_LINK_ADR_REQ( )                      US915_CASE { return RegionUS915LinkAdrReq( linkAdrReq, drOut, txPowOut, nbRepOut, nbBytesParsed ); }
#define US915_RX_PARAM_SETUP_REQ( )                US915_CASE { return RegionUS915RxParamSetupReq( rxParamSetupReq ); }
#define US915_NEW_CHANNEL_REQ( )                   US915_CASE { return RegionUS915NewChannelReq( newChannelReq ); }
#define US915_TX_PARAM_SETUP_REQ( )                US915_CASE { return RegionUS915TxParamSetupReq( txParamSetupReq ); }
#define US915_DL_CHANNEL_REQ( )                    US915_CASE { return RegionUS915DlChannelReq( dlChannelReq ); }
#define US915_ALTERNATE_DR( )                      US915_CASE { return RegionUS915AlternateDr( alternateDr ); }
#define US915_CALC_BACKOFF( )                      US915_CASE { RegionUS915CalcBackOff( calcBackOff ); break; }
#define US915_NEXT_CHANNEL( )                      US915_CASE { return RegionUS915NextChannel( nextChanParams, channel, time, aggregatedTimeOff ); }
#define US915_CHANNEL_ADD( )                       US915_CASE { return RegionUS915ChannelAdd( channelAdd ); }
#define US915_CHANNEL_REMOVE( )                    US915_CASE { return RegionUS915ChannelsRemove( channelRemove ); }
#define US915_SET_CONTINUOUS_WAVE( )               US915_CASE { RegionUS915SetContinuousWave( continuousWave ); break; }
#define US915_APPLY_DR_OFFSET( )                   US915_CASE { return RegionUS915ApplyDrOffset( downlinkDwellTime, dr, drOffset ); }
#define US915_RX_BEACON_SETUP( )                   US915_CASE { RegionUS915RxBeaconSetup( rxBeaconSetup, outDr ); }


bool RegionIsActive( LoRaMacRegion_t region )
{
    switch( region )
    {
        AS923_IS_ACTIVE( );
        AU915_IS_ACTIVE( );
        CN470_IS_ACTIVE( );
        CN779_IS_ACTIVE( );
        EU433_IS_ACTIVE( );
        EU868_IS_ACTIVE( );
        KR920_IS_ACTIVE( );
        IN865_IS_ACTIVE( );
        US915_IS_ACTIVE( );
        US915_HYBRID_IS_ACTIVE( );
        default:
        {
            return false;
        }
    }
}

PhyParam_t RegionGetPhyParam( LoRaMacRegion_t region, GetPhyParams_t* getPhy )
{
    PhyParam_t phyParam = { 0 };
    switch( region )
    {
        AS923_GET_PHY_PARAM( );
        AU915_GET_PHY_PARAM( );
        CN470_GET_PHY_PARAM( );
        CN779_GET_PHY_PARAM( );
        EU433_GET_PHY_PARAM( );
        EU868_GET_PHY_PARAM( );
        KR920_GET_PHY_PARAM( );
        IN865_GET_PHY_PARAM( );
        US915_GET_PHY_PARAM( );
        US915_HYBRID_GET_PHY_PARAM( );
        default:
        {
            return phyParam;
        }
    }
}

void RegionSetBandTxDone( LoRaMacRegion_t region, SetBandTxDoneParams_t* txDone )
{
    switch( region )
    {
        AS923_SET_BAND_TX_DONE( );
        AU915_SET_BAND_TX_DONE( );
        CN470_SET_BAND_TX_DONE( );
        CN779_SET_BAND_TX_DONE( );
        EU433_SET_BAND_TX_DONE( );
        EU868_SET_BAND_TX_DONE( );
        KR920_SET_BAND_TX_DONE( );
        IN865_SET_BAND_TX_DONE( );
        US915_SET_BAND_TX_DONE( );
        US915_HYBRID_SET_BAND_TX_DONE( );
        default:
        {
            return;
        }
    }
}

void RegionInitDefaults( LoRaMacRegion_t region, InitType_t type )
{
    switch( region )
    {
        AS923_INIT_DEFAULTS( );
        AU915_INIT_DEFAULTS( );
        CN470_INIT_DEFAULTS( );
        CN779_INIT_DEFAULTS( );
        EU433_INIT_DEFAULTS( );
        EU868_INIT_DEFAULTS( );
        KR920_INIT_DEFAULTS( );
        IN865_INIT_DEFAULTS( );
        US915_INIT_DEFAULTS( );
        US915_HYBRID_INIT_DEFAULTS( );
        default:
        {
            break;
        }
    }
}

bool RegionVerify( LoRaMacRegion_t region, VerifyParams_t* verify, PhyAttribute_t phyAttribute )
{
    switch( region )
    {
        AS923_VERIFY( );
        AU915_VERIFY( );
        CN470_VERIFY( );
        CN779_VERIFY( );
        EU433_VERIFY( );
        EU868_VERIFY( );
        KR920_VERIFY( );
        IN865_VERIFY( );
        US915_VERIFY( );
        US915_HYBRID_VERIFY( );
        default:
        {
            return false;
        }
    }
}

void RegionApplyCFList( LoRaMacRegion_t region, ApplyCFListParams_t* applyCFList )
{
    switch( region )
    {
        AS923_APPLY_CF_LIST( );
        AU915_APPLY_CF_LIST( );
        CN470_APPLY_CF_LIST( );
        CN779_APPLY_CF_LIST( );
        EU433_APPLY_CF_LIST( );
        EU868_APPLY_CF_LIST( );
        KR920_APPLY_CF_LIST( );
        IN865_APPLY_CF_LIST( );
        US915_APPLY_CF_LIST( );
        US915_HYBRID_APPLY_CF_LIST( );
        default:
        {
            break;
        }
    }
}

bool RegionChanMaskSet( LoRaMacRegion_t region, ChanMaskSetParams_t* chanMaskSet )
{
    switch( region )
    {
        AS923_CHAN_MASK_SET( );
        AU915_CHAN_MASK_SET( );
        CN470_CHAN_MASK_SET( );
        CN779_CHAN_MASK_SET( );
        EU433_CHAN_MASK_SET( );
        EU868_CHAN_MASK_SET( );
        KR920_CHAN_MASK_SET( );
        IN865_CHAN_MASK_SET( );
        US915_CHAN_MASK_SET( );
        US915_HYBRID_CHAN_MASK_SET( );
        default:
        {
            return false;
        }
    }
}

bool RegionAdrNext( LoRaMacRegion_t region, AdrNextParams_t* adrNext, int8_t* drOut, int8_t* txPowOut, uint32_t* adrAckCounter )
{
    switch( region )
    {
        AS923_ADR_NEXT( );
        AU915_ADR_NEXT( );
        CN470_ADR_NEXT( );
        CN779_ADR_NEXT( );
        EU433_ADR_NEXT( );
        EU868_ADR_NEXT( );
        KR920_ADR_NEXT( );
        IN865_ADR_NEXT( );
        US915_ADR_NEXT( );
        US915_HYBRID_ADR_NEXT( );
        default:
        {
            return false;
        }
    }
}

void RegionComputeRxWindowParameters( LoRaMacRegion_t region, int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams )
{
    switch( region )
    {
        AS923_COMPUTE_RX_WINDOW_PARAMETERS( );
        AU915_COMPUTE_RX_WINDOW_PARAMETERS( );
        CN470_COMPUTE_RX_WINDOW_PARAMETERS( );
        CN779_COMPUTE_RX_WINDOW_PARAMETERS( );
        EU433_COMPUTE_RX_WINDOW_PARAMETERS( );
        EU868_COMPUTE_RX_WINDOW_PARAMETERS( );
        KR920_COMPUTE_RX_WINDOW_PARAMETERS( );
        IN865_COMPUTE_RX_WINDOW_PARAMETERS( );
        US915_COMPUTE_RX_WINDOW_PARAMETERS( );
        US915_HYBRID_COMPUTE_RX_WINDOW_PARAMETERS( );
        default:
        {
            break;
        }
    }
}

bool RegionRxConfig( LoRaMacRegion_t region, RxConfigParams_t* rxConfig, int8_t* datarate )
{
    switch( region )
    {
        AS923_RX_CONFIG( );
        AU915_RX_CONFIG( );
        CN470_RX_CONFIG( );
        CN779_RX_CONFIG( );
        EU433_RX_CONFIG( );
        EU868_RX_CONFIG( );
        KR920_RX_CONFIG( );
        IN865_RX_CONFIG( );
        US915_RX_CONFIG( );
        US915_HYBRID_RX_CONFIG( );
        default:
        {
            return false;
        }
    }
}

bool RegionTxConfig( LoRaMacRegion_t region, TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    switch( region )
    {
        AS923_TX_CONFIG( );
        AU915_TX_CONFIG( );
        CN470_TX_CONFIG( );
        CN779_TX_CONFIG( );
        EU433_TX_CONFIG( );
        EU868_TX_CONFIG( );
        KR920_TX_CONFIG( );
        IN865_TX_CONFIG( );
        US915_TX_CONFIG( );
        US915_HYBRID_TX_CONFIG( );
        default:
        {
            return false;
        }
    }
}

uint8_t RegionLinkAdrReq( LoRaMacRegion_t region, LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed )
{
    switch( region )
    {
        AS923_LINK_ADR_REQ( );
        AU915_LINK_ADR_REQ( );
        CN470_LINK_ADR_REQ( );
        CN779_LINK_ADR_REQ( );
        EU433_LINK_ADR_REQ( );
        EU868_LINK_ADR_REQ( );
        KR920_LINK_ADR_REQ( );
        IN865_LINK_ADR_REQ( );
        US915_LINK_ADR_REQ( );
        US915_HYBRID_LINK_ADR_REQ( );
        default:
        {
            return 0;
        }
    }
}

uint8_t RegionRxParamSetupReq( LoRaMacRegion_t region, RxParamSetupReqParams_t* rxParamSetupReq )
{
    switch( region )
    {
        AS923_RX_PARAM_SETUP_REQ( );
        AU915_RX_PARAM_SETUP_REQ( );
        CN470_RX_PARAM_SETUP_REQ( );
        CN779_RX_PARAM_SETUP_REQ( );
        EU433_RX_PARAM_SETUP_REQ( );
        EU868_RX_PARAM_SETUP_REQ( );
        KR920_RX_PARAM_SETUP_REQ( );
        IN865_RX_PARAM_SETUP_REQ( );
        US915_RX_PARAM_SETUP_REQ( );
        US915_HYBRID_RX_PARAM_SETUP_REQ( );
        default:
        {
            return 0;
        }
    }
}

uint8_t RegionNewChannelReq( LoRaMacRegion_t region, NewChannelReqParams_t* newChannelReq )
{
    switch( region )
    {
        AS923_NEW_CHANNEL_REQ( );
        AU915_NEW_CHANNEL_REQ( );
        CN470_NEW_CHANNEL_REQ( );
        CN779_NEW_CHANNEL_REQ( );
        EU433_NEW_CHANNEL_REQ( );
        EU868_NEW_CHANNEL_REQ( );
        KR920_NEW_CHANNEL_REQ( );
        IN865_NEW_CHANNEL_REQ( );
        US915_NEW_CHANNEL_REQ( );
        US915_HYBRID_NEW_CHANNEL_REQ( );
        default:
        {
            return 0;
        }
    }
}

int8_t RegionTxParamSetupReq( LoRaMacRegion_t region, TxParamSetupReqParams_t* txParamSetupReq )
{
    switch( region )
    {
        AS923_TX_PARAM_SETUP_REQ( );
        AU915_TX_PARAM_SETUP_REQ( );
        CN470_TX_PARAM_SETUP_REQ( );
        CN779_TX_PARAM_SETUP_REQ( );
        EU433_TX_PARAM_SETUP_REQ( );
        EU868_TX_PARAM_SETUP_REQ( );
        KR920_TX_PARAM_SETUP_REQ( );
        IN865_TX_PARAM_SETUP_REQ( );
        US915_TX_PARAM_SETUP_REQ( );
        US915_HYBRID_TX_PARAM_SETUP_REQ( );
        default:
        {
            return 0;
        }
    }
}

uint8_t RegionDlChannelReq( LoRaMacRegion_t region, DlChannelReqParams_t* dlChannelReq )
{
    switch( region )
    {
        AS923_DL_CHANNEL_REQ( );
        AU915_DL_CHANNEL_REQ( );
        CN470_DL_CHANNEL_REQ( );
        CN779_DL_CHANNEL_REQ( );
        EU433_DL_CHANNEL_REQ( );
        EU868_DL_CHANNEL_REQ( );
        KR920_DL_CHANNEL_REQ( );
        IN865_DL_CHANNEL_REQ( );
        US915_DL_CHANNEL_REQ( );
        US915_HYBRID_DL_CHANNEL_REQ( );
        default:
        {
            return 0;
        }
    }
}

int8_t RegionAlternateDr( LoRaMacRegion_t region, AlternateDrParams_t* alternateDr )
{
    switch( region )
    {
        AS923_ALTERNATE_DR( );
        AU915_ALTERNATE_DR( );
        CN470_ALTERNATE_DR( );
        CN779_ALTERNATE_DR( );
        EU433_ALTERNATE_DR( );
        EU868_ALTERNATE_DR( );
        KR920_ALTERNATE_DR( );
        IN865_ALTERNATE_DR( );
        US915_ALTERNATE_DR( );
        US915_HYBRID_ALTERNATE_DR( );
        default:
        {
            return 0;
        }
    }
}

void RegionCalcBackOff( LoRaMacRegion_t region, CalcBackOffParams_t* calcBackOff )
{
    switch( region )
    {
        AS923_CALC_BACKOFF( );
        AU915_CALC_BACKOFF( );
        CN470_CALC_BACKOFF( );
        CN779_CALC_BACKOFF( );
        EU433_CALC_BACKOFF( );
        EU868_CALC_BACKOFF( );
        KR920_CALC_BACKOFF( );
        IN865_CALC_BACKOFF( );
        US915_CALC_BACKOFF( );
        US915_HYBRID_CALC_BACKOFF( );
        default:
        {
            break;
        }
    }
}

bool RegionNextChannel( LoRaMacRegion_t region, NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff )
{
    switch( region )
    {
        AS923_NEXT_CHANNEL( );
        AU915_NEXT_CHANNEL( );
        CN470_NEXT_CHANNEL( );
        CN779_NEXT_CHANNEL( );
        EU433_NEXT_CHANNEL( );
        EU868_NEXT_CHANNEL( );
        KR920_NEXT_CHANNEL( );
        IN865_NEXT_CHANNEL( );
        US915_NEXT_CHANNEL( );
        US915_HYBRID_NEXT_CHANNEL( );
        default:
        {
            return false;
        }
    }
}

LoRaMacStatus_t RegionChannelAdd( LoRaMacRegion_t region, ChannelAddParams_t* channelAdd )
{
    switch( region )
    {
        AS923_CHANNEL_ADD( );
        AU915_CHANNEL_ADD( );
        CN470_CHANNEL_ADD( );
        CN779_CHANNEL_ADD( );
        EU433_CHANNEL_ADD( );
        EU868_CHANNEL_ADD( );
        KR920_CHANNEL_ADD( );
        IN865_CHANNEL_ADD( );
        US915_CHANNEL_ADD( );
        US915_HYBRID_CHANNEL_ADD( );
        default:
        {
            return LORAMAC_STATUS_PARAMETER_INVALID;
        }
    }
}

bool RegionChannelsRemove( LoRaMacRegion_t region, ChannelRemoveParams_t* channelRemove )
{
    switch( region )
    {
        AS923_CHANNEL_REMOVE( );
        AU915_CHANNEL_REMOVE( );
        CN470_CHANNEL_REMOVE( );
        CN779_CHANNEL_REMOVE( );
        EU433_CHANNEL_REMOVE( );
        EU868_CHANNEL_REMOVE( );
        KR920_CHANNEL_REMOVE( );
        IN865_CHANNEL_REMOVE( );
        US915_CHANNEL_REMOVE( );
        US915_HYBRID_CHANNEL_REMOVE( );
        default:
        {
            return false;
        }
    }
}

void RegionSetContinuousWave( LoRaMacRegion_t region, ContinuousWaveParams_t* continuousWave )
{
    switch( region )
    {
        AS923_SET_CONTINUOUS_WAVE( );
        AU915_SET_CONTINUOUS_WAVE( );
        CN470_SET_CONTINUOUS_WAVE( );
        CN779_SET_CONTINUOUS_WAVE( );
        EU433_SET_CONTINUOUS_WAVE( );
        EU868_SET_CONTINUOUS_WAVE( );
        KR920_SET_CONTINUOUS_WAVE( );
        IN865_SET_CONTINUOUS_WAVE( );
        US915_SET_CONTINUOUS_WAVE( );
        US915_HYBRID_SET_CONTINUOUS_WAVE( );
        default:
        {
            break;
        }
    }
}

uint8_t RegionApplyDrOffset( LoRaMacRegion_t region, uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset )
{
    switch( region )
    {
        AS923_APPLY_DR_OFFSET( );
        AU915_APPLY_DR_OFFSET( );
        CN470_APPLY_DR_OFFSET( );
        CN779_APPLY_DR_OFFSET( );
        EU433_APPLY_DR_OFFSET( );
        EU868_APPLY_DR_OFFSET( );
        KR920_APPLY_DR_OFFSET( );
        IN865_APPLY_DR_OFFSET( );
        US915_APPLY_DR_OFFSET( );
        US915_HYBRID_APPLY_DR_OFFSET( );
        default:
        {
            return dr;
        }
    }
}

void RegionRxBeaconSetup( LoRaMacRegion_t region, RxBeaconSetup_t* rxBeaconSetup, uint8_t* outDr )
{
    switch( region )
    {
        AS923_RX_BEACON_SETUP( );
        AU915_RX_BEACON_SETUP( );
        CN470_RX_BEACON_SETUP( );
        CN779_RX_BEACON_SETUP( );
        EU433_RX_BEACON_SETUP( );
        EU868_RX_BEACON_SETUP( );
        KR920_RX_BEACON_SETUP( );
        IN865_RX_BEACON_SETUP( );
        US915_RX_BEACON_SETUP( );
        US915_HYBRID_RX_BEACON_SETUP( );
        default:
        {
            break;
        }
    }
}
