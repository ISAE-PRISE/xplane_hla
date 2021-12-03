//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// XPLANE HLA PLUGIN
//
// Copyright (C) 2018-2022  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email:  jean-baptiste.chaudron@isae-supaero.fr
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------


// System includes
#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#else
#include <Windows.h>
#endif
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

// Xplane includes
#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMMenus.h"
#include "XPWidgetUtils.h"
#include "XPUIGraphics.h"

#if IBM
#include <windows.h>
#endif
#if LIN
#include <GL/gl.h>
#else
#if __GNUC__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

#ifndef XPLM301
    #error This is made to be compiled against the XPLM301 SDK
#endif

#define LISTBOX_ITEM_HEIGHT 12

// Local includes
#include <XplaneFederateHla1516e.hh>

// Enums for x-plane native colors.
enum {

    xpColor_MenuDarkTinge = 0,
    xpColor_MenuBkgnd,
    xpColor_MenuHilite,
    xpColor_MenuLiteTinge,
    xpColor_MenuText,
    xpColor_MenuTextDisabled,
    xpColor_SubTitleText,
    xpColor_TabFront,
    xpColor_TabBack,
    xpColor_CaptionText,
    xpColor_ListText,
    xpColor_GlassText,
    xpColor_Count
};

enum {
    // This is the item number of the current item, starting at 0.
    xpProperty_ListBoxCurrentItem					= 1900,
    // This will add an item to the list box at the end.
    xpProperty_ListBoxAddItem						= 1901,
    // This will clear the list box and then add the items.
    xpProperty_ListBoxAddItemsWithClear				= 1902,
    // This will clear the list box.
    xpProperty_ListBoxClear							= 1903,
    // This will insert an item into the list box at the index.
    xpProperty_ListBoxInsertItem					= 1904,
    // This will delete an item from the list box at the index.
    xpProperty_ListBoxDeleteItem					= 1905,
    // This stores the pointer to the listbox data.
    xpProperty_ListBoxData							= 1906,
    // This stores the max Listbox Items.
    xpProperty_ListBoxMaxListBoxItems				= 1907,
    // This stores the highlight state.
    xpProperty_ListBoxHighlighted					= 1908,
    // This stores the scrollbar Min.
    xpProperty_ListBoxScrollBarMin					= 1909,
    // This stores the scrollbar Max.
    xpProperty_ListBoxScrollBarMax					= 1910,
    // This stores the scrollbar SliderPosition.
    xpProperty_ListBoxScrollBarSliderPosition		= 1911,
    // This stores the scrollbar ScrollBarPageAmount.
    xpProperty_ListBoxScrollBarPageAmount			= 1912
};

enum {
    // This message is sent when an item is picked.
    // param 1 is the widget that was picked, param 2
    // is the item number.
    xpMessage_ListBoxItemSelected				= 1900
};

// Enums for the datarefs we get them from.
static const char *	kXPlaneColorNames[] = {
    "sim/graphics/colors/menu_dark_rgb",
    "sim/graphics/colors/menu_bkgnd_rgb",
    "sim/graphics/colors/menu_hilite_rgb",
    "sim/graphics/colors/menu_lite_rgb",
    "sim/graphics/colors/menu_text_rgb",
    "sim/graphics/colors/menu_text_disabled_rgb",
    "sim/graphics/colors/subtitle_text_rgb",
    "sim/graphics/colors/tab_front_rgb",
    "sim/graphics/colors/tab_back_rgb",
    "sim/graphics/colors/caption_text_rgb",
    "sim/graphics/colors/list_text_rgb",
    "sim/graphics/colors/glass_text_rgb"
};


// Those datarefs are only XP7; if we can't find one,
// fall back to this table of X-Plane 6 colors.
static const float	kBackupColors[xpColor_Count][3] =
{
     { (const float)(33.0/256.0), (const float)(41.0/256.0), (const float)(44.0/256.0) },
     { (const float)(53.0/256.0), (const float)(64.0/256.0), (const float)(68.0/256.0) },
     { (const float)(65.0/256.0), (const float)(83.0/256.0), (const float)(89.0/256.0) },
     { (const float)(65.0/256.0), (const float)(83.0/256.0), (const float)(89.0/256.0) },
     { (const float)0.8, (const float)0.8, (const float)0.8 },
     { (const float)0.4, (const float)0.4, (const float)0.4 }
};

// This array contains the resolved datarefs
static XPLMDataRef	gColorRefs[xpColor_Count];

// Current alpha levels to blit at.
static float		gAlphaLevel = 1.0;

std::unique_ptr<XplaneFederateHla1516e> federate;

static unsigned int g_nbOtherPlaneToDraw = 0;

static std::string g_planeSelected = "";

static bool g_menuIHLAIsCreated{false},
            g_menuItemFlightDynamicsIsCreated{false},
            g_menuItemHydraulicActuatorsIsCreated{false},
            g_menuItemEngineActuatorsIsCreated{false},
            g_menuVolFormationIsCreated{false};

static bool g_menuFlightDynamicsIsVisible{false},
            g_menuHydraulicActuatorsIsVisible{false},
            g_menuEngineActuatorsIsVisible{false},
            g_menuVolFormationIsVisible{false};

int g_menu_container_idx; // The index of our menu item in the Plugins menu
XPLMMenuID g_menu_id; // The menu container we'll append all our menu items to
static XPWidgetID g_hlaWidgetsWidget, g_hlaWidgetsWindow;
static XPWidgetID g_disableJoystick, g_enableJoystick, g_joinFederationButton, g_allFederateReady, g_disconnectFederationButton, g_forcePositionButton, g_forcePositionButton2;
static XPWidgetID g_widgetListLogs;
static XPWidgetID g_labelOtherPlanesToDraw, g_widgetListOtherPlanes;
static XPWidgetID g_labelOtherPlaneDescription, g_widgetListOtherPlaneDescription;

static XPWidgetID g_myPlaneFlightDynamicsWidgetsWidget, g_myPlaneFlightDynamicsWidgetsWindow, g_labelMyPlaneFlightDynamicsModel, g_widgetMyPlaneFlightDynamicsModel;
static XPWidgetID g_labelMyPlaneTimeInformationModelData, g_widgetMyPlaneTimeInformationModelData;
static XPWidgetID g_myPlaneHydraulicActuatorsWidgetsWidget, g_myPlaneHydraulicActuatorsWidgetsWindow, g_labelMyPlaneHydraulicActuatorsModel, g_widgetMyPlaneHydraulicActuatorsModel;
static XPWidgetID g_myPlaneEngineActuatorsWidgetsWidget, g_myPlaneEngineActuatorsWidgetsWindow, g_labelMyPlaneEngineActuatorsModel, g_widgetMyPlaneEngineActuatorsModel;
static XPWidgetID g_myPlaneVolFormationWidgetsWidget, g_myPlaneVolFormationWidgetsWindow, g_widgetMyPlaneVolFormationModel;
static XPWidgetID g_labelLongitudeModel, g_textFieldLongitudeModel;
static XPWidgetID g_labelLatitudeModel, g_textFieldLatitudeModel;
static XPWidgetID g_labelAltitudeModel, g_textFieldAltitudeModel;
static XPWidgetID g_labelDistanceModel, g_textFieldDistanceModel;
static XPWidgetID g_labelVitesseModel, g_textFieldVitesseModel;
static XPWidgetID g_labelSeparatorModel;
static XPWidgetID g_labelCapModel, g_textFieldCapModel;
static XPWidgetID g_labelVelocityXModel, g_textFieldVelocityXModel;
static XPWidgetID g_labelVelocityYModel, g_textFieldVelocityYModel;
static XPWidgetID g_labelVelocityZModel, g_textFieldVelocityZModel;
static XPWidgetID g_volFormationButtonInitialize, g_volFormationButtonInitialize2;
static XPWidgetID g_labelFormationModel, g_listFormationModel;


/**
 * @brief The XPListBoxData_t struct
 *              This structure represents a listbox internally...it consists of arrays
 *              per item and some general stuff.
 */
struct	XPListBoxData_t {
    //!< Per item:
    std::vector<std::string>	Items;		//!< The name of the item
    std::vector<int>			Lefts;		//!< The rectangle of the item, relative to the top left corner of the listbox/
    std::vector<int>			Rights;
};

static XPListBoxData_t *pListBoxData;

/**
 * @brief XPListBoxGetItemNumber
 *          This routine finds the item that is in a given point, or returns -1 if there is none.
 *          It simply trolls through all the items.
 * @param pListBoxData
 * @param inX
 * @param inY
 * @return
 */
static int XPListBoxGetItemNumber(XPListBoxData_t * pListBoxData, int inX, int inY)
{
    for (unsigned int n = 0; n < pListBoxData->Items.size(); ++n)
    {
        if ((inX >= pListBoxData->Lefts[n]) && (inX < pListBoxData->Rights[n]) &&
            (inY >= (n * LISTBOX_ITEM_HEIGHT)) && (inY < ((n * LISTBOX_ITEM_HEIGHT) + LISTBOX_ITEM_HEIGHT)))
        {
            return n;
        }
    }
    return -1;
}

/**
 * @brief SetupAmbientColor
 *          This routine sets up a color from the above table.  Pass
 *          in a float[3] to get the color; pass in NULL to have the
 *          OpenGL color be set immediately.
 * @param inColorID
 * @param outColor
 */
static void	SetupAmbientColor(int inColorID, float * outColor)
{
    // If we're running the first time, resolve all of our datarefs just once.
    static	bool	firstTime = true;
    if (firstTime)
    {
        firstTime = false;
        for (int n = 0; n <xpColor_Count; ++n)
        {
            gColorRefs[n] = XPLMFindDataRef(kXPlaneColorNames[n]);
        }
    }

    // If being asked to set the color immediately, allocate some storage.
    float	theColor[4];
    float * target = outColor ? outColor : theColor;

    // If we have a dataref, just fetch the color from the ref.
    if (gColorRefs[inColorID])
        XPLMGetDatavf(gColorRefs[inColorID], target, 0, 3);
    else {

        // If we didn't have a dataref, fetch the ambient cabin lighting,
        // since XP6 dims the UI with night.
        static	XPLMDataRef	ambient_r = XPLMFindDataRef("sim/graphics/misc/cockpit_light_level_r");
        static	XPLMDataRef	ambient_g = XPLMFindDataRef("sim/graphics/misc/cockpit_light_level_g");
        static	XPLMDataRef	ambient_b = XPLMFindDataRef("sim/graphics/misc/cockpit_light_level_b");

        // Use a backup color but dim it.
        target[0] = kBackupColors[inColorID][0] * XPLMGetDataf(ambient_r);
        target[1] = kBackupColors[inColorID][1] * XPLMGetDataf(ambient_g);
        target[2] = kBackupColors[inColorID][2] * XPLMGetDataf(ambient_b);
    }

    // If the user passed NULL, set the color now using the alpha level.
    if (!outColor)
    {
        theColor[3] = gAlphaLevel;
        glColor4fv(theColor);
    }
}

/**
 * @brief XPListBoxFillWithData
 *          This widget Proc implements the actual listbox.
 * @param pListBoxData
 * @param inItems
 * @param Width
 */
static void XPListBoxFillWithData(XPListBoxData_t *pListBoxData, const char *inItems, int Width)
{
    std::string	Items(inItems);
    while (!Items.empty())
    {
        std::string::size_type split = Items.find(';');
        if (split == Items.npos)
        {
            split = Items.size();
        }

        std::string	Item = Items.substr(0, split);

        pListBoxData->Items.push_back(Item);
        pListBoxData->Lefts.push_back(0);
        pListBoxData->Rights.push_back(Width);

        if (Item.size() == Items.size())
            break;
        else
            Items = Items.substr(split+1);
    }
}

/**
 * @brief XPListBoxAddItem
 * @param pListBoxData
 * @param pBuffer
 * @param Width
 */
static void XPListBoxAddItem(XPListBoxData_t *pListBoxData, char *pBuffer, int Width)
{
    std::string	Item(pBuffer);

    pListBoxData->Items.push_back(Item);
    pListBoxData->Lefts.push_back(0);
    pListBoxData->Rights.push_back(Width);
}

/**
 * @brief XPListBoxClear
 * @param pListBoxData
 */
static void XPListBoxClear(XPListBoxData_t *pListBoxData)
{
    pListBoxData->Items.clear();
    pListBoxData->Lefts.clear();
    pListBoxData->Rights.clear();
}

/**
 * @brief XPListBoxInsertItem
 * @param pListBoxData
 * @param pBuffer
 * @param Width
 * @param CurrentItem
 */
static void XPListBoxInsertItem(XPListBoxData_t *pListBoxData, char *pBuffer, int Width, int CurrentItem)
{
    std::string	Item(pBuffer);

    pListBoxData->Items.insert(pListBoxData->Items.begin() + CurrentItem, Item);
    pListBoxData->Lefts.insert(pListBoxData->Lefts.begin() + CurrentItem, 0);
    pListBoxData->Rights.insert(pListBoxData->Rights.begin() + CurrentItem, Width);
}

/**
 * @brief XPListBoxDeleteItem
 * @param pListBoxData
 * @param CurrentItem
 */
static void XPListBoxDeleteItem(XPListBoxData_t *pListBoxData, int CurrentItem)
{
    pListBoxData->Items.erase(pListBoxData->Items.begin() + CurrentItem);
    pListBoxData->Lefts.erase(pListBoxData->Lefts.begin() + CurrentItem);
    pListBoxData->Rights.erase(pListBoxData->Rights.begin() + CurrentItem);
}


/**
 * @brief XPListBoxProc
 *          This method create list box and handle all messages inside
 * @param inMessage Id of the message
 * @param inWidget Id of the widget
 * @param inParam1 Custom paramter 1
 * @param inParam2 Custom paramter 2
 * @return 1 if message is handled, 0 otherwise
 */
static int		XPListBoxProc(
                    XPWidgetMessage			inMessage,
                    XPWidgetID				inWidget,
                    intptr_t				inParam1,
                    intptr_t				inParam2)
{
    static int ScrollBarSlop;

    // Select if we're in the background.
    if (XPUSelectIfNeeded(inMessage, inWidget, inParam1, inParam2, 1/*eat*/))	return 1;

    int Left, Top, Right, Bottom, x, y, ListBoxDataOffset, ListBoxIndex;
    char Buffer[4096];

    int IsVertical, DownBtnSize, DownPageSize, ThumbSize, UpPageSize, UpBtnSize;
    bool UpBtnSelected, DownBtnSelected, ThumbSelected, UpPageSelected, DownPageSelected;

    XPGetWidgetGeometry(inWidget, &Left, &Top, &Right, &Bottom);

    int	SliderPosition = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, NULL);
    int	Min = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMin, NULL);
    int	Max = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, NULL);
    int	ScrollBarPageAmount = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarPageAmount, NULL);
    int	CurrentItem = XPGetWidgetProperty(inWidget, xpProperty_ListBoxCurrentItem, NULL);
    int	MaxListBoxItems = XPGetWidgetProperty(inWidget, xpProperty_ListBoxMaxListBoxItems, NULL);
    int	Highlighted = XPGetWidgetProperty(inWidget, xpProperty_ListBoxHighlighted, NULL);
    XPListBoxData_t	*pListBoxData = (XPListBoxData_t*) XPGetWidgetProperty(inWidget, xpProperty_ListBoxData, NULL);

    switch(inMessage)
    {
        case xpMsg_Create:
            // Allocate mem for the structure.
            pListBoxData = new XPListBoxData_t;
            XPGetWidgetDescriptor(inWidget, Buffer, sizeof(Buffer));
            XPListBoxFillWithData(pListBoxData, Buffer, (Right - Left - 20));
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxData, (intptr_t)pListBoxData);
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxCurrentItem, 0);
            Min = 0;
            Max = pListBoxData->Items.size();
            ScrollBarSlop = 0;
            Highlighted = false;
            SliderPosition = Max;
            MaxListBoxItems = (Top - Bottom) / LISTBOX_ITEM_HEIGHT;
            ScrollBarPageAmount = MaxListBoxItems;
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMin, Min);
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, Max);
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarPageAmount, ScrollBarPageAmount);
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxMaxListBoxItems, MaxListBoxItems);
            XPSetWidgetProperty(inWidget, xpProperty_ListBoxHighlighted, Highlighted);
            return 1;

        case xpMsg_DescriptorChanged:
            return 1;

        case xpMsg_PropertyChanged:
            if (XPGetWidgetProperty(inWidget, xpProperty_ListBoxAddItem, NULL))
            {
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxAddItem, 0);
                XPGetWidgetDescriptor(inWidget, Buffer, sizeof(Buffer));
                XPListBoxAddItem(pListBoxData, Buffer, (Right - Left - 20));
                Max = pListBoxData->Items.size();
                SliderPosition = Max;
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, Max);
            }

            if (XPGetWidgetProperty(inWidget, xpProperty_ListBoxAddItemsWithClear, NULL))
            {
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxAddItemsWithClear, 0);
                XPGetWidgetDescriptor(inWidget, Buffer, sizeof(Buffer));
                XPListBoxClear(pListBoxData);
                XPListBoxFillWithData(pListBoxData, Buffer, (Right - Left - 20));
                Max = pListBoxData->Items.size();
                SliderPosition = Max;
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, Max);
            }

            if (XPGetWidgetProperty(inWidget, xpProperty_ListBoxClear, NULL))
            {
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxClear, 0);
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxCurrentItem, 0);
                XPListBoxClear(pListBoxData);
                Max = pListBoxData->Items.size();
                SliderPosition = Max;
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, Max);
            }

            if (XPGetWidgetProperty(inWidget, xpProperty_ListBoxInsertItem, NULL))
            {
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxInsertItem, 0);
                XPGetWidgetDescriptor(inWidget, Buffer, sizeof(Buffer));
                XPListBoxInsertItem(pListBoxData, Buffer, (Right - Left - 20), CurrentItem);
            }

            if (XPGetWidgetProperty(inWidget, xpProperty_ListBoxDeleteItem, NULL))
            {
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxDeleteItem, 0);
                if ((pListBoxData->Items.size() > 0) && (pListBoxData->Items.size() > static_cast<unsigned long>(CurrentItem)))
                    XPListBoxDeleteItem(pListBoxData, CurrentItem);
            }
            return 1;

        case xpMsg_Draw:
        {
            int	x, y;
            XPLMGetMouseLocation(&x, &y);

            XPDrawWindow(Left, Bottom, Right-20, Top, xpWindow_ListView);
            XPDrawTrack(Right-20, Bottom, Right, Top, Min, Max, SliderPosition, xpTrack_ScrollBar, Highlighted);

            XPLMSetGraphicsState(0, 1, 0,  0, 1,  0, 0);
            XPLMBindTexture2d(XPLMGetTexture(xplm_Tex_GeneralInterface), 0);
            glColor4f(1.0, 1.0, 1.0, 1.0);

            unsigned int ItemNumber;
            XPLMSetGraphicsState(0, 0, 0,  0, 0,  0, 0);

            // Now draw each item.
            ListBoxIndex = Max - SliderPosition;
            ItemNumber = 0;
            while (ItemNumber < static_cast<unsigned int>(MaxListBoxItems))
            {
                if (ListBoxIndex < static_cast<int>(pListBoxData->Items.size()))
                {
                    // Calculate the item rect in global coordinates.
                    int ItemTop    = Top - (ItemNumber * LISTBOX_ITEM_HEIGHT);
                    int ItemBottom = Top - ((ItemNumber * LISTBOX_ITEM_HEIGHT) + LISTBOX_ITEM_HEIGHT);

                    // If we are hilited, draw the hilite bkgnd.
                    if (CurrentItem == ListBoxIndex)
                    {
                        gAlphaLevel = 0.25;
                        XPLMSetGraphicsState(0, 0, 0,  0, 1, 0, 0);
                        SetupAmbientColor(xpColor_MenuHilite, NULL);
                        gAlphaLevel = 1.0;
                        glBegin(GL_QUADS);
                        glVertex2i(Left, ItemTop);
                        glVertex2i(Right-20, ItemTop);
                        glVertex2i(Right-20, ItemBottom);
                        glVertex2i(Left, ItemBottom);
                        glEnd();
                    }

                    float	text[3];
                    SetupAmbientColor(xpColor_ListText, text);

                    char	Buffer[512];
                    int		FontWidth, FontHeight;
                    int		ListBoxWidth = (Right - 20) - Left;
                    strcpy(Buffer, pListBoxData->Items[ListBoxIndex++].c_str());
                    XPLMGetFontDimensions(xplmFont_Basic, &FontWidth, &FontHeight, NULL);
                    int		MaxChars = ListBoxWidth / FontWidth;
                    Buffer[MaxChars] = 0;

                    XPLMDrawString(text,
                                Left, ItemBottom + 2,
                                const_cast<char *>(Buffer), NULL, xplmFont_Basic);
                }
                ItemNumber++;
            }
        }
            return 1;

        case xpMsg_MouseUp:
            if (IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, Top, Right, Bottom))
            {
                Highlighted = false;
                XPSetWidgetProperty(inWidget, xpProperty_ListBoxHighlighted, Highlighted);
            }

            if (IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Left, Top, Right-20, Bottom))
            {
                if (pListBoxData->Items.size() > 0)
                {
                    if (CurrentItem != -1)
                        XPSetWidgetDescriptor(inWidget, pListBoxData->Items[CurrentItem].c_str());
                    else
                        XPSetWidgetDescriptor(inWidget, "");
                    XPSendMessageToWidget(inWidget, xpMessage_ListBoxItemSelected, xpMode_UpChain, (intptr_t) inWidget, (intptr_t) CurrentItem);
                }
            }
            return 1;

        case xpMsg_MouseDown:
            if (IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Left, Top, Right-20, Bottom))
            {
                if (pListBoxData->Items.size() > 0)
                {
                    XPLMGetMouseLocation(&x, &y);
                    ListBoxDataOffset = XPListBoxGetItemNumber(pListBoxData, x - Left, Top - y);
                    if (ListBoxDataOffset != -1)
                    {
                        ListBoxDataOffset += (Max - SliderPosition);
                        if (ListBoxDataOffset < static_cast<int>(pListBoxData->Items.size()))
                            XPSetWidgetProperty(inWidget, xpProperty_ListBoxCurrentItem, ListBoxDataOffset);
                    }
                }
            }

            if (IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, Top, Right, Bottom))
            {
                XPGetTrackMetrics(Right-20, Bottom, Right, Top, Min, Max, SliderPosition, xpTrack_ScrollBar, &IsVertical, &DownBtnSize, &DownPageSize, &ThumbSize, &UpPageSize, &UpBtnSize);
                int	Min = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMin, NULL);
                int	Max = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, NULL);
                if (IsVertical)
                {
                    UpBtnSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, Top, Right, Top - UpBtnSize);
                    DownBtnSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, Bottom + DownBtnSize, Right, Bottom);
                    UpPageSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, (Top - UpBtnSize), Right, (Bottom + DownBtnSize + DownPageSize + ThumbSize));
                    DownPageSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, (Top - UpBtnSize - UpPageSize - ThumbSize), Right, (Bottom + DownBtnSize));
                    ThumbSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, (Top - UpBtnSize - UpPageSize), Right, (Bottom + DownBtnSize + DownPageSize));
                }
                else
                {
                    DownBtnSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, Top, Right-20 + UpBtnSize, Bottom);
                    UpBtnSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20 - DownBtnSize, Top, Right, Bottom);
                    DownPageSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20 + DownBtnSize, Top, Right - UpBtnSize - UpPageSize - ThumbSize, Bottom);
                    UpPageSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20 + DownBtnSize + DownPageSize + ThumbSize, Top, Right - UpBtnSize, Bottom);
                    ThumbSelected = IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20 + DownBtnSize + DownPageSize, Top, Right - UpBtnSize - UpPageSize, Bottom);
                }

                if (UpPageSelected)
                {
                    SliderPosition+=ScrollBarPageAmount;
                    if (SliderPosition > Max)
                        SliderPosition = Max;
                    XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                }
                else if (DownPageSelected)
                {
                    SliderPosition-=ScrollBarPageAmount;
                    if (SliderPosition < Min)
                        SliderPosition = Min;
                    XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                }
                else if (UpBtnSelected)
                {
                    SliderPosition++;
                    if (SliderPosition > Max)
                        SliderPosition = Max;
                    XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                }
                else if (DownBtnSelected)
                {
                    SliderPosition--;
                    if (SliderPosition < Min)
                        SliderPosition = Min;
                    XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
                }
                else if (ThumbSelected)
                {
                    if (IsVertical)
                        ScrollBarSlop = Bottom + DownBtnSize + DownPageSize + (ThumbSize/2) - MOUSE_Y(inParam1);
                    else
                        ScrollBarSlop = Right-20 + DownBtnSize + DownPageSize + (ThumbSize/2) - MOUSE_X(inParam1);
                    Highlighted = true;
                    XPSetWidgetProperty(inWidget, xpProperty_ListBoxHighlighted, Highlighted);

                }
                else
                {
                    Highlighted = false;
                    XPSetWidgetProperty(inWidget, xpProperty_ListBoxHighlighted, Highlighted);
                }
            }
        return 1;

    case xpMsg_MouseDrag:
        if (IN_RECT(MOUSE_X(inParam1), MOUSE_Y(inParam1), Right-20, Top, Right, Bottom))
        {
            XPGetTrackMetrics(Right-20, Bottom, Right, Top, Min, Max, SliderPosition, xpTrack_ScrollBar, &IsVertical, &DownBtnSize, &DownPageSize, &ThumbSize, &UpPageSize, &UpBtnSize);
            int	Min = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMin, NULL);
            int	Max = XPGetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarMax, NULL);

            ThumbSelected = Highlighted;

            if (ThumbSelected)
            {
                if (inParam1 != 0)
                {
                    if (IsVertical)
                    {
                        y = MOUSE_Y(inParam1) + ScrollBarSlop;
                        SliderPosition = round((float)((float)(y - (Bottom + DownBtnSize + ThumbSize/2)) /
                                    (float)((Top - UpBtnSize - ThumbSize/2) - (Bottom + DownBtnSize + ThumbSize/2))) * Max);
                    }
                    else
                    {
                        x = MOUSE_X(inParam1) + ScrollBarSlop;
                        SliderPosition = round((float)((float)(x - (Right-20 + DownBtnSize + ThumbSize/2)) / (float)((Right - UpBtnSize - ThumbSize/2) - (Right-20 + DownBtnSize + ThumbSize/2))) * Max);
                    }

                }
                else
                    SliderPosition = 0;

                if (SliderPosition < Min)
                    SliderPosition = Min;
                if (SliderPosition > Max)
                    SliderPosition = Max;

                XPSetWidgetProperty(inWidget, xpProperty_ListBoxScrollBarSliderPosition, SliderPosition);
            }
        }
        return 1;

        default:
            return 0;
    }
}

/**
 * @brief addItemInListLog
 * @param item The item to add
 */
void addItemInListLog(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetListLogs, item.c_str());
    XPSetWidgetProperty(g_widgetListLogs, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief clearListOtherPlanes
 */
void clearListOtherPlanes()
{
    XPSetWidgetProperty(g_widgetListOtherPlanes, xpProperty_ListBoxClear, 1);
}

/**
 * @brief addItemInListOtherPlanes
 * @param item The item to add
 */
void addItemInListOtherPlanes(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetListOtherPlanes, item.c_str());
    XPSetWidgetProperty(g_widgetListOtherPlanes, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief addItemInListOtherPlaneDescription
 * @param item The item to add
 */
void addItemInListOtherPlaneDescription(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetListOtherPlaneDescription, item.c_str());
    XPSetWidgetProperty(g_widgetListOtherPlaneDescription, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief clearListOtherPlaneDescription
 */
void clearListOtherPlaneDescription()
{
    XPSetWidgetProperty(g_widgetListOtherPlaneDescription, xpProperty_ListBoxClear, 1);
}

/**
 * @brief addItemInListFormation
 * @param item The Item to add
 */
void addItemInListFormation(const std::string &item)
{
    XPSetWidgetDescriptor(g_listFormationModel, item.c_str());
    XPSetWidgetProperty(g_listFormationModel, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief updateAircraftsItems
 *          Update aircraft data in FlightDynmaic widget
 * @param planeName The name of the plane to display data
 */
void updateAircraftsItems(const std::string& planeName)
{
    if(federate)
    {
        auto itAircraft = federate->getOthersAircraftToDraw().find(XplaneFederateHla1516e::getWString(planeName.c_str()));
        if(itAircraft != federate->getOthersAircraftToDraw().end())
        {
            auto aircraft = itAircraft->second;
            clearListOtherPlaneDescription();
            addItemInListOtherPlaneDescription("name      : \t" + aircraft.aircraft_name);
            addItemInListOtherPlaneDescription("n°        : \t" + std::to_string(aircraft.aircraftNo));
            addItemInListOtherPlaneDescription("type      : \t" + aircraft.aircraft_type);
            addItemInListOtherPlaneDescription("elevation : \t" + std::to_string(aircraft.plane_el));
            addItemInListOtherPlaneDescription("latitude  : \t" + std::to_string(aircraft.plane_lat));
            addItemInListOtherPlaneDescription("longitude : \t" + std::to_string(aircraft.plane_lon));
            addItemInListOtherPlaneDescription("phi       : \t" + std::to_string(aircraft.plane_phi));
            addItemInListOtherPlaneDescription("psi       : \t" + std::to_string(aircraft.plane_psi));
            addItemInListOtherPlaneDescription("theta     : \t" + std::to_string(aircraft.plane_the));
            addItemInListOtherPlaneDescription("velocity X: \t" + std::to_string(aircraft.plane_vx));
            addItemInListOtherPlaneDescription("velocity Y: \t" + std::to_string(aircraft.plane_vy));
            addItemInListOtherPlaneDescription("velocity Z: \t" + std::to_string(aircraft.plane_vz));
            addItemInListOtherPlaneDescription("throttle  : \t" + std::to_string(aircraft.plane_throttle[0]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[1]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[2]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[3]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[4]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[5]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[6]));
            addItemInListOtherPlaneDescription("            \t" + std::to_string(aircraft.plane_throttle[7]));
        }
    }
}

/**
 * @brief addFlightDynamicData
 * @param item The itemù to add
 */
void addFlightDynamicData(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetMyPlaneFlightDynamicsModel, item.c_str());
    XPSetWidgetProperty(g_widgetMyPlaneFlightDynamicsModel, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief clearFlightDynamicData
 */
void clearFlightDynamicData()
{
    XPSetWidgetProperty(g_widgetMyPlaneFlightDynamicsModel, xpProperty_ListBoxClear, 1);
}

/**
 * @brief addEngineActuatorData
 * @param item The item to add
 */
void addEngineActuatorData(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetMyPlaneEngineActuatorsModel, item.c_str());
    XPSetWidgetProperty(g_widgetMyPlaneEngineActuatorsModel, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief clearEngineActuatorData
 */
void clearEngineActuatorData()
{
    XPSetWidgetProperty(g_widgetMyPlaneEngineActuatorsModel, xpProperty_ListBoxClear, 1);
}

/**
 * @brief addHydraulicActuatorData
 * @param item The item to add
 */
void addHydraulicActuatorData(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetMyPlaneHydraulicActuatorsModel, item.c_str());
    XPSetWidgetProperty(g_widgetMyPlaneHydraulicActuatorsModel, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief clearHydraulicActuatorData
 */
void clearHydraulicActuatorData()
{
    XPSetWidgetProperty(g_widgetMyPlaneHydraulicActuatorsModel, xpProperty_ListBoxClear, 1);
}

/**
 * @brief addTimeInformationData
 * @param item the item to add
 */
void addTimeInformationData(const std::string &item)
{
    XPSetWidgetDescriptor(g_widgetMyPlaneTimeInformationModelData, item.c_str());
    XPSetWidgetProperty(g_widgetMyPlaneTimeInformationModelData, xpProperty_ListBoxAddItem, 1);
}

/**
 * @brief clearTimeInformationData
 */
void clearTimeInformationData()
{
    XPSetWidgetProperty(g_widgetMyPlaneTimeInformationModelData, xpProperty_ListBoxClear, 1);
}

/**
 * @brief updateFlightDynamicData
 *          update data in FlightDynamic widget from the federate
 */
void updateFlightDynamicData()
{
    if(federate)
    {
        clearFlightDynamicData();
        HLAfixedRecord localHLAFlightDynamicsModelData = federate->getLocalHLAFlightDynamicsModelData();
        addFlightDynamicData("latitutdeDeg          : " + std::to_string(static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(0))));
        addFlightDynamicData("longitudeDeg          : " + std::to_string(static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(1))));
        addFlightDynamicData("altitudeMSLMeters     : " + std::to_string(static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(2))));
        addFlightDynamicData("altitudeAGLMeters     : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(3))));
        addFlightDynamicData("phiDeg                : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(4))));
        addFlightDynamicData("thetaDeg              : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(5))));
        addFlightDynamicData("psiDeg                : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(6))));
        addFlightDynamicData("alphaDeg              : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(7))));
        addFlightDynamicData("betaDeg               : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(8))));
        addFlightDynamicData("hpathDeg              : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(9))));
        addFlightDynamicData("vpathDeg              : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(10))));
        addFlightDynamicData("pDegps                : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(11))));
        addFlightDynamicData("qDegps                : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(12))));
        addFlightDynamicData("rDegps                : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(13))));
        addFlightDynamicData("trueAirspeedMetersps  : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(14))));
        addFlightDynamicData("indicatedAirspeedKias : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(15))));
        addFlightDynamicData("groundspeedMetersps   : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(16))));
        addFlightDynamicData("verticalspeedMetersps : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(17))));
        addFlightDynamicData("vMach                 : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(18))));
        addFlightDynamicData("rho                   : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(19))));

        addFlightDynamicData("localX                : " + std::to_string(static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(20))));
        addFlightDynamicData("localY                : " + std::to_string(static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(21))));
        addFlightDynamicData("localZ                : " + std::to_string(static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(22))));

        addFlightDynamicData("localVxMetersps       : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(23))));
        addFlightDynamicData("localVyMetersps       : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(24))));
        addFlightDynamicData("localVzMetersps       : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(25))));
        addFlightDynamicData("localAxMetersps2      : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(26))));
        addFlightDynamicData("localAyMetersps2      : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(27))));
        addFlightDynamicData("localAzMetersps2      : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(28))));

    }
}

/**
 * @brief updateEngineActuatorsData
 *          update data in EngineActuator widget from the federate
 */
void updateEngineActuatorsData()
{
    if(federate)
    {
        clearEngineActuatorData();

        HLAfixedRecord localHLAEngineActuatorsModelData = federate->getLocalHLAEngineActuatorsModelData();

        HLAfixedArray arrayThrottle = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(0));
        for(unsigned int i=0; i < arrayThrottle.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("Trottle : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayThrottle.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayThrottle.get(i))));
        }

        HLAfixedArray arrayThurst = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(1));
        for(unsigned int i=0; i < arrayThurst.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("Thurst  : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayThurst.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayThurst.get(i))));
        }

        HLAfixedArray arrayN1 = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(2));
        for(unsigned int i=0; i < arrayN1.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("N1      : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayN1.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayN1.get(i))));
        }

        HLAfixedArray arrayN2 = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(3));
        for(unsigned int i=0; i < arrayN2.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("N2      : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayN2.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayN2.get(i))));
        }

        HLAfixedArray arrayEGT = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(4));
        for(unsigned int i=0; i < arrayEGT.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("EGT     : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayEGT.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayEGT.get(i))));
        }

        HLAfixedArray arrayEPR = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(5));
        for(unsigned int i=0; i < arrayEPR.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("EPR     : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayEPR.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayEPR.get(i))));
        }

        HLAfixedArray arrayFF = static_cast<const HLAfixedArray&>(localHLAEngineActuatorsModelData.get(6));
        for(unsigned int i=0; i < arrayFF.size(); i++)
        {
            if(i==0)
                addEngineActuatorData("FF      : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayFF.get(i))));
            else
                addEngineActuatorData("          " + std::to_string(static_cast<const HLAfloat32LE&>(arrayFF.get(i))));
        }

    }
}

/**
 * @brief updateHydraulicActuatorData
 *          update data in hydraulic actuator widget from the federate
 */
void updateHydraulicActuatorData()
{
    if(federate)
    {
        clearHydraulicActuatorData();

        HLAfixedRecord localHLAHydraulicActuatorModelData = federate->getLocalHLAHydraulicActuatorsModelData();
        addHydraulicActuatorData("LeftAileron          : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(0))));
        addHydraulicActuatorData("RightAileron         : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(1))));
        addHydraulicActuatorData("LeftElevator         : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(2))));
        addHydraulicActuatorData("RightElevator        : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(3))));
        addHydraulicActuatorData("HorizontalStabilizer : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(4))));
        addHydraulicActuatorData("Flaps                : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(5))));
        addHydraulicActuatorData("Rudder               : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(6))));
        addHydraulicActuatorData("LeftSpeedbrakes      : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(7))));
        addHydraulicActuatorData("RightSpeedbrakes     : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(8))));
        addHydraulicActuatorData("LeftSpoilers         : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(9))));
        addHydraulicActuatorData("RightSpoilers        : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLAHydraulicActuatorModelData.get(10))));

        HLAfixedArray arrayGEARS = static_cast<const HLAfixedArray&>(localHLAHydraulicActuatorModelData.get(11));
        for(unsigned int i=0; i < arrayGEARS.size(); i++)
        {
            if(i==0)
                addHydraulicActuatorData("Gears                : " + std::to_string(static_cast<const HLAfloat32LE&>(arrayGEARS.get(i))));
            else
                addHydraulicActuatorData("                       " + std::to_string(static_cast<const HLAfloat32LE&>(arrayGEARS.get(i))));
        }
    }
}

/**
 * @brief updateTimeInformationData
 *          update Time information in FlightDynamic widget from the federate
 */
void updateTimeInformationData()
{
    if(federate)
    {
        clearTimeInformationData();

        HLAfixedRecord localHLATimeInformationModelData = federate->getLocalHLATimeInformationModelData();
        addTimeInformationData("Date day          : " + std::to_string(static_cast<const HLAinteger32LE&>(localHLATimeInformationModelData.get(0))));
        addTimeInformationData("Local time in sec : " + std::to_string(static_cast<const HLAfloat32LE&>(localHLATimeInformationModelData.get(1))));
    }
}

/**
 * @brief updateVolFormationData
 *          update data in FormationFlight widget from the federate
 */
void updateVolFormationData()
{
    if(federate)
    {
        HLAfixedRecord localHLAFlightDynamicsModelData = federate->getLocalHLAFlightDynamicsModelData();

        std::ostringstream latitude;
        latitude << static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(0)).get();
        XPSetWidgetDescriptor(g_textFieldLatitudeModel, latitude.str().c_str());

        std::ostringstream longitude;
        longitude << static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(1)).get();
        XPSetWidgetDescriptor(g_textFieldLongitudeModel, longitude.str().c_str());

        std::ostringstream altitude;
        altitude << static_cast<const HLAfloat64LE&>(localHLAFlightDynamicsModelData.get(2)).get();
        XPSetWidgetDescriptor(g_textFieldAltitudeModel, altitude.str().c_str());

        std::ostringstream cap;
        cap << static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(6)).get();
        XPSetWidgetDescriptor(g_textFieldCapModel, cap.str().c_str());

        std::ostringstream velocityX;
        velocityX << static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(23)).get();
        XPSetWidgetDescriptor(g_textFieldVelocityXModel, velocityX.str().c_str());

        std::ostringstream velocityY;
        velocityY << static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(24)).get();
        XPSetWidgetDescriptor(g_textFieldVelocityYModel, velocityY.str().c_str());

        std::ostringstream velocityZ;
        velocityZ << static_cast<const HLAfloat32LE&>(localHLAFlightDynamicsModelData.get(25)).get();
        XPSetWidgetDescriptor(g_textFieldVelocityZModel, velocityZ.str().c_str());
    }
}

/**
 * @brief updateVolFormationData2
 *          update data in FormationFlight widget from the federate
 */
void updateVolFormationData2()
{
    if(federate)
    {
        // Search for selected aircraft in list
        RemoteAircraftToDraw aircraft = federate->getOthersAircraftToDraw().begin()->second;

        std::ostringstream latitude;
        latitude << aircraft.plane_lat;
        XPSetWidgetDescriptor(g_textFieldLatitudeModel, latitude.str().c_str());

        std::ostringstream longitude;
        longitude << aircraft.plane_lon;
        XPSetWidgetDescriptor(g_textFieldLongitudeModel, longitude.str().c_str());

        std::ostringstream altitude;
        altitude << aircraft.plane_el;
        XPSetWidgetDescriptor(g_textFieldAltitudeModel, altitude.str().c_str());

        std::ostringstream cap;
        cap << aircraft.plane_psi;
        XPSetWidgetDescriptor(g_textFieldCapModel, cap.str().c_str());

        std::ostringstream velocityX;
        velocityX << aircraft.plane_vx;
        XPSetWidgetDescriptor(g_textFieldVelocityXModel, velocityX.str().c_str());

        std::ostringstream velocityY;
        velocityY << aircraft.plane_vy;
        XPSetWidgetDescriptor(g_textFieldVelocityYModel, velocityY.str().c_str());

        std::ostringstream velocityZ;
        velocityZ << aircraft.plane_vz;
        XPSetWidgetDescriptor(g_textFieldVelocityZModel, velocityZ.str().c_str());
    }
}

/**
 * @brief myPlaneFlightDynamicsWidgetsHandler
 *          Handle all messages in Flight dynamic widget.
 *          For this case only one message is handled, it's close widget message
 * @param inMessage The id of the type of the message
 * @param inWidget Not used
 * @param inParam1 Not used
 * @param inParam2 Not used
 * @return 1 if a message is handled, 0 otherwise
 */
int	myPlaneFlightDynamicsWidgetsHandler(
                        XPWidgetMessage			inMessage,
                        XPWidgetID				/*inWidget*/,
                        intptr_t				/*inParam1*/,
                        intptr_t				/*inParam2*/)
{
    // Close button pressed, only hide the widget, rather than destropying it.
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        if (g_menuItemFlightDynamicsIsCreated == 1)
        {
            XPHideWidget(g_myPlaneFlightDynamicsWidgetsWidget);
            g_menuFlightDynamicsIsVisible = false;
        }
        return 1;
    }
    return 0;
}

/**
 * @brief myPlaneHydraulicActuatorsWidgetsHandler
 *          Handle all messages in Vol formation widget.
 *          For this case only one message is handled, it's close widget message
 * @param inMessage The id of the type of the message
 * @param inWidget Not used
 * @param inParam1 Not used
 * @param inParam2 Not used
 * @return 1 if a message is handled, 0 otherwise
 */
int	myPlaneHydraulicActuatorsWidgetsHandler(
                        XPWidgetMessage			inMessage,
                        XPWidgetID				/*inWidget*/,
                        intptr_t				/*inParam1*/,
                        intptr_t				/*inParam2*/)
{
    // Close button pressed, only hide the widget, rather than destropying it.
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        if (g_menuItemHydraulicActuatorsIsCreated == 1)
        {
            XPHideWidget(g_myPlaneHydraulicActuatorsWidgetsWidget);
            g_menuHydraulicActuatorsIsVisible = false;
        }
        return 1;
    }
    return 0;
}

/**
 * @brief volFormationWidgetsHandler
 *          Handle all messages in Vol formation widget.
 *          For this case only one message is handled, it's close widget message
 * @param inMessage The id of the type of the message
 * @param inWidget Not used
 * @param inParam1 Not used
 * @param inParam2 Not used
 * @return 1 if a message is handled, 0 otherwise
 */
int	myPlaneEngineActuatorsWidgetsHandler(
                        XPWidgetMessage			inMessage,
                        XPWidgetID				/*inWidget*/,
                        intptr_t				/*inParam1*/,
                        intptr_t				/*inParam2*/)
{
    // Close button pressed, only hide the widget, rather than destropying it.
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        if (g_menuItemEngineActuatorsIsCreated == 1)
        {
            XPHideWidget(g_myPlaneEngineActuatorsWidgetsWidget);
            g_menuEngineActuatorsIsVisible = false;
        }
        return 1;
    }
    return 0;
}

/**
 * @brief volFormationWidgetsHandler
 *          Handle all messages in Flight formation widget
 * @param inMessage The id of the type of the message
 * @param inWidget Not used
 * @param inParam1 The id of the message
 * @param inParam2 Not used
 * @return 1 if a message is handled, 0 otherwise
 */
int	volFormationWidgetsHandler(
                        XPWidgetMessage			inMessage,
                        XPWidgetID				/*inWidget*/,
                        intptr_t				inParam1,
                        intptr_t				/*inParam2*/)
{

    // Close button pressed, only hide the widget, rather than destropying it.
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        if (g_menuVolFormationIsCreated == 1)
        {
            XPHideWidget(g_myPlaneVolFormationWidgetsWidget);
            g_menuVolFormationIsVisible = false;
        }
        return 1;
    }
    else if (inMessage == xpMsg_PushButtonPressed)
    {
        // This test set the popup index to the number
        // entered into the PopupInputTextEdit edit box
        if (inParam1 == (intptr_t)g_volFormationButtonInitialize)
        {
            if(federate && federate->isInitialized())
            {

                updateVolFormationData();

                char Buffer1[512];
                XPGetWidgetDescriptor(g_textFieldLongitudeModel, Buffer1, sizeof(Buffer1));
                double longitude = atof(Buffer1);

                char Buffer2[512];
                XPGetWidgetDescriptor(g_textFieldLatitudeModel, Buffer2, sizeof(Buffer2));
                double latitude = atof(Buffer2);

                char Buffer3[512];
                XPGetWidgetDescriptor(g_textFieldAltitudeModel, Buffer3, sizeof(Buffer3));
                double altitude = atof(Buffer3);

                char Buffer4[512];
                XPGetWidgetDescriptor(g_textFieldDistanceModel, Buffer4, sizeof(Buffer4));
                double distance = atof(Buffer4);

                char Buffer5[512];
                XPGetWidgetDescriptor(g_textFieldVitesseModel, Buffer5, sizeof(Buffer5));
                double vitesse = atof(Buffer5);

                char Buffer6[512];
                XPGetWidgetDescriptor(g_textFieldCapModel, Buffer6, sizeof(Buffer6));
                double cap = atof(Buffer6);

                char Buffer7[512];
                XPGetWidgetDescriptor(g_listFormationModel, Buffer7, sizeof(Buffer7));
                std::string formationStr(Buffer7);

                char Buffer8[512];
                XPGetWidgetDescriptor(g_textFieldVelocityXModel, Buffer8, sizeof(Buffer8));
                float velocityX = atof(Buffer8);

                char Buffer9[512];
                XPGetWidgetDescriptor(g_textFieldVelocityYModel, Buffer9, sizeof(Buffer9));
                float velocityY = atof(Buffer9);

                char Buffer10[512];
                XPGetWidgetDescriptor(g_textFieldVelocityZModel, Buffer10, sizeof(Buffer10));
                float velocityZ = atof(Buffer10);


                std::cout << formationStr << std::endl;
                std::wstring formation(XplaneFederateHla1516e::getWString(formationStr.c_str()));

                std::wcout << formation << std::endl;
                std::wcout << longitude << " - " << latitude << " - " << altitude << " - " << distance << " - " << vitesse << " - " << cap << " - " << formation << std::endl;
                Formation enumFormation;
                if(formationStr == "Colonne")
                    enumFormation = Formation::_column;
                else if(formationStr == "Ligne")
                    enumFormation = Formation::_line;
                else if(formationStr == "Echelon droit")
                    enumFormation = Formation::_rightEchelon;
                else if(formationStr == "Echelon gauche")
                    enumFormation = Formation::_leftEchelon;
                else if(formationStr == "Triangle")
                    enumFormation = Formation::_triangular;

                federate->sendInteractionsVolFormation(latitude, longitude, altitude, distance, velocityX, velocityY, velocityZ, cap, enumFormation);
            }
        }// This test set the popup index to the number
        // entered into the PopupInputTextEdit edit box
        if (inParam1 == (intptr_t)g_volFormationButtonInitialize2)
        {
            if(federate && federate->isInitialized())
            {
                updateVolFormationData2();

                char Buffer1[512];
                XPGetWidgetDescriptor(g_textFieldLongitudeModel, Buffer1, sizeof(Buffer1));
                double longitude = atof(Buffer1);

                char Buffer2[512];
                XPGetWidgetDescriptor(g_textFieldLatitudeModel, Buffer2, sizeof(Buffer2));
                double latitude = atof(Buffer2);

                char Buffer3[512];
                XPGetWidgetDescriptor(g_textFieldAltitudeModel, Buffer3, sizeof(Buffer3));
                double altitude = atof(Buffer3);

                char Buffer4[512];
                XPGetWidgetDescriptor(g_textFieldDistanceModel, Buffer4, sizeof(Buffer4));
                double distance = atof(Buffer4);

                char Buffer5[512];
                XPGetWidgetDescriptor(g_textFieldVitesseModel, Buffer5, sizeof(Buffer5));
                double vitesse = atof(Buffer5);

                char Buffer6[512];
                XPGetWidgetDescriptor(g_textFieldCapModel, Buffer6, sizeof(Buffer6));
                double cap = atof(Buffer6);

                char Buffer7[512];
                XPGetWidgetDescriptor(g_listFormationModel, Buffer7, sizeof(Buffer7));
                std::string formationStr(Buffer7);

                char Buffer8[512];
                XPGetWidgetDescriptor(g_textFieldVelocityXModel, Buffer8, sizeof(Buffer8));
                float velocityX = atof(Buffer8);

                char Buffer9[512];
                XPGetWidgetDescriptor(g_textFieldVelocityYModel, Buffer9, sizeof(Buffer9));
                float velocityY = atof(Buffer9);

                char Buffer10[512];
                XPGetWidgetDescriptor(g_textFieldVelocityZModel, Buffer10, sizeof(Buffer10));
                float velocityZ = atof(Buffer10);


                std::cout << formationStr << std::endl;
                std::wstring formation(XplaneFederateHla1516e::getWString(formationStr.c_str()));

                std::wcout << formation << std::endl;
                std::wcout << longitude << " - " << latitude << " - " << altitude << " - " << distance << " - " << vitesse << " - " << cap << " - " << formation << std::endl;
                Formation enumFormation;
                if(formationStr == "Colonne")
                    enumFormation = Formation::_column;
                else if(formationStr == "Ligne")
                    enumFormation = Formation::_line;
                else if(formationStr == "Echelon droit")
                    enumFormation = Formation::_rightEchelon;
                else if(formationStr == "Echelon gauche")
                    enumFormation = Formation::_leftEchelon;
                else if(formationStr == "Triangle")
                    enumFormation = Formation::_triangular;

                federate->sendInteractionsVolFormation2(latitude, longitude, altitude, distance, velocityX, velocityY, velocityZ, cap, enumFormation);
            }
        }
        return 1;
    }

    return 0;
}

/**
 * @brief HlaWidgetsHandler
 *          Handle all messages in HLA widget
 * @param inMessage The id of the type of the message
 * @param inWidget Not used
 * @param inParam1 The id of the message
 * @param inParam2 Not used
 * @return 1 if a message is handled, 0 otherwise
 */
int	HlaWidgetsHandler(
                        XPWidgetMessage			inMessage,
                        XPWidgetID				/*inWidget*/,
                        intptr_t				inParam1,
                        intptr_t				/*inParam2*/)
{
    char Buffer[1024];

    // Close button pressed, only hide the widget, rather than destropying it.
    if (inMessage == xpMessage_CloseButtonPushed)
    {
        if (g_menuIHLAIsCreated == 1)
        {
            XPHideWidget(g_hlaWidgetsWidget);
        }
        return 1;
    }

    // Test for a button pressed
    if (inMessage == xpMsg_PushButtonPressed)
    {
        // This test set the popup index to the number
        // entered into the PopupInputTextEdit edit box
        if (inParam1 == (intptr_t)g_joinFederationButton)
        {
            try {
                addItemInListLog("Federate creation");
                char path[512];
                char type[256];
                XPLMGetNthAircraftModel(0, type, path);
                std::wstring aircraft_type = XplaneFederateHla1516e::getWString(type);
                std::wstring nameFederate = L"XPLANE_" + std::to_wstring(rand()) + L"-" + aircraft_type;
                addItemInListLog("Federate connection");
                federate->connect(L"Federation", nameFederate, L"isae_prise_hla1516e.xml");
                addItemInListLog("Federate connected");
                addItemInListLog("Federation creation");
                federate->createFederationExecution();
                if(federate->isCreator()) {
                    addItemInListLog("Federation created");
                }
                else {
                    addItemInListLog("Federation already created");
                }
                addItemInListLog("Join Federation");
                federate->joinFederationExecution();
                addItemInListLog("Federation joined");
                addItemInListLog("Get all handles");
                federate->getAllHandles();
                addItemInListLog("All handles got");
                addItemInListLog("Publish and subscribe");
                federate->publishAndSubscribe();
                addItemInListLog("Publish and subscribe done");
                addItemInListLog("Register object instance");
                federate->registerObjectInstances();
                addItemInListLog("Register object instance done");


                federate->setHlaTimeManagementSettings(1, 0.1, 0.0, 4000000);
                federate->enableTimeRegulation();
                federate->enableTimeConstrained();
                federate->enableAsynchronousDelivery();
                if(!federate->isCreator()) {
                    federate->pause();
                    federate->setInitialized(true);
                }

                XPHideWidget(g_joinFederationButton);
                if(federate->isCreator()) {
                    XPShowWidget(g_allFederateReady);
                    XPHideWidget(g_disconnectFederationButton);
                    XPHideWidget(g_forcePositionButton);
                    XPHideWidget(g_forcePositionButton2);
                    XPHideWidget(g_labelOtherPlanesToDraw);
                    XPHideWidget(g_widgetListOtherPlanes);
                    XPHideWidget(g_labelOtherPlaneDescription);
                    XPHideWidget(g_widgetListOtherPlaneDescription);
                }
                else {
                    XPHideWidget(g_allFederateReady);
                    XPShowWidget(g_disconnectFederationButton);
                    XPShowWidget(g_forcePositionButton);
                    XPShowWidget(g_forcePositionButton2);
                    XPShowWidget(g_labelOtherPlanesToDraw);
                    XPShowWidget(g_widgetListOtherPlanes);
                    XPShowWidget(g_labelOtherPlaneDescription);
                    XPShowWidget(g_widgetListOtherPlaneDescription);
                }

            } catch (const rti1516e::Exception &e) {
                addItemInListLog("Error after the federate creation!" + XplaneFederateHla1516e::getString(e.what()));
            } catch (...) {
                addItemInListLog("Error after the federate creation!");
            }

            return 1;
        }
        else if (inParam1 == (intptr_t)g_allFederateReady)
        {
            if(federate)
            {
                federate->pause();
                federate->setInitialized(true);
                XPHideWidget(g_allFederateReady);
                if(federate->getOthersAircraftToDraw().size() == 0)
                    XPShowWidget(g_disconnectFederationButton);
                XPShowWidget(g_forcePositionButton);
                XPShowWidget(g_forcePositionButton2);
                XPShowWidget(g_labelOtherPlanesToDraw);
                XPShowWidget(g_widgetListOtherPlanes);
                XPShowWidget(g_labelOtherPlaneDescription);
                XPShowWidget(g_widgetListOtherPlaneDescription);
            }
            return 1;
        }
        else if (inParam1 == (intptr_t)g_disconnectFederationButton)
        {
            if(federate)
            {
                federate->setInitialized(false);
                addItemInListLog("Unpublish and Unsubscribe");
                federate->unpublishAndUnsubscribe();
                addItemInListLog("Unpublish and Unsubscribe Done");
                addItemInListLog("Delete object instances");
                federate->deleteObjectInstances();
                addItemInListLog("Delete object instances Done");
                addItemInListLog("Resign federation");
                federate->resignFederationExecution();
                addItemInListLog("Resign federation Done");
                addItemInListLog("Destroy federation");
                federate->destroyFederationExecution();

                XPHideWidget(g_disconnectFederationButton);
                XPHideWidget(g_forcePositionButton);
                XPHideWidget(g_forcePositionButton2);

                clearListOtherPlanes();
                XPHideWidget(g_labelOtherPlanesToDraw);
                XPHideWidget(g_widgetListOtherPlanes);

                clearListOtherPlaneDescription();
                g_planeSelected = "";
                XPHideWidget(g_labelOtherPlaneDescription);
                XPHideWidget(g_widgetListOtherPlaneDescription);
                XPHideWidget(g_allFederateReady);
                XPShowWidget(g_joinFederationButton);
                federate.reset();
                federate = std::make_unique<XplaneFederateHla1516e>();
                std::cout << "End of disconnect" << std::endl;
            }
            return 1;
        }
        else if (inParam1 == (intptr_t)g_forcePositionButton)
        {
            if(federate)
            {
                federate->sendInteractionForcePosition();
                federate->printOtherPlanes();
            }
            return 1;
        }
        else if (inParam1 == (intptr_t)g_forcePositionButton2)
        {
            if(federate && g_planeSelected != "")
            {
                // Search for selected aircraft in list
                auto itAircraft = federate->getOthersAircraftToDraw().find(XplaneFederateHla1516e::getWString(g_planeSelected.c_str()));
                if(itAircraft != federate->getOthersAircraftToDraw().end())
                {
                    auto aircraft = itAircraft->second;

                    HLAfloat64LE hlaLatitude(aircraft.plane_lat);
                    HLAfloat64LE hlaLongitude(aircraft.plane_lon);
                    HLAfloat64LE hlaAltitude(aircraft.plane_el);

                    // Force current aircraft position to selected aircraft position
                    federate->setToXplaneLocalPosition(hlaLatitude, hlaLongitude, hlaAltitude);
                }
            }
            return 1;
        }
        else if (inParam1 == (intptr_t)g_disableJoystick)
        {
            if(federate)
            {
                federate->disableJoystick();
                XPHideWidget(g_disableJoystick);
                XPShowWidget(g_enableJoystick);
            }
        }
        else if (inParam1 == (intptr_t)g_enableJoystick)
        {
            if(federate)
            {
                federate->enableJoystick();
                XPHideWidget(g_enableJoystick);
                XPShowWidget(g_disableJoystick);
            }
        }
    }
    else if (inMessage == xpMessage_ListBoxItemSelected)
    {
        XPGetWidgetDescriptor(g_widgetListOtherPlanes, Buffer, sizeof(Buffer));
        std::string planeName(Buffer);
        g_planeSelected = planeName;
        return 1;
    }

    return 0;
}

/**
 * @brief CreateHlaWidgets
 *          Create HLA widget to connect to federation and force porition of other plane
 * @param x Coord X
 * @param y Coord Y
 * @param w Width
 * @param h Height
 */
void CreateHlaWidgets(int x, int y, int w, int h)
{
    int x2 = x + w;
    int y2 = y - h;

    int dy = -50;

    g_hlaWidgetsWidget = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "Hla Widget",	// desc
                    1,		// root
                    NULL,	// no container
                    xpWidgetClass_MainWindow);

    XPSetWidgetProperty(g_hlaWidgetsWidget, xpProperty_MainWindowType, 1);
    XPSetWidgetProperty(g_hlaWidgetsWidget, xpProperty_MainWindowHasCloseBoxes, 1);

    g_hlaWidgetsWindow = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "",	// desc
                    0,		// root
                    g_hlaWidgetsWidget,
                    xpWidgetClass_SubWindow);

    XPSetWidgetProperty(g_hlaWidgetsWindow, xpProperty_SubWindowType, xpSubWindowStyle_SubWindow);

    g_disableJoystick = XPCreateWidget(x+150, y-70-dy, x+250, y-102-dy,
                    1, " Disable joystick", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);

    XPSetWidgetProperty(g_disableJoystick, xpProperty_ButtonType, xpPushButton);
    XPHideWidget(g_disableJoystick);

    g_enableJoystick = XPCreateWidget(x+150, y-70-dy, x+250, y-102-dy,
                    1, " Enable joystick", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);

    XPSetWidgetProperty(g_enableJoystick, xpProperty_ButtonType, xpPushButton);
    XPHideWidget(g_enableJoystick);

    g_joinFederationButton = XPCreateWidget(x+150, y-110-dy, x+250, y-132-dy,
                    1, " Join federation", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);

    XPSetWidgetProperty(g_joinFederationButton, xpProperty_ButtonType, xpPushButton);

    g_allFederateReady = XPCreateWidget(x+140, y-110-dy, x+260, y-132-dy,
                    1, " All federates ready", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);

    XPSetWidgetProperty(g_allFederateReady, xpProperty_ButtonType, xpPushButton);
    XPHideWidget(g_allFederateReady);

    g_disconnectFederationButton = XPCreateWidget(x+130, y-110-dy, x+270, y-132-dy,
                    1, " Disconnect federation", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);

    XPSetWidgetProperty(g_disconnectFederationButton, xpProperty_ButtonType, xpPushButton);
    XPHideWidget(g_disconnectFederationButton);

    g_forcePositionButton = XPCreateWidget(x+30, y-140-dy, x+170, y-162-dy,
                    1, " Force position", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);
    g_forcePositionButton2 = XPCreateWidget(x+200, y-140-dy, x+340, y-162-dy,
                    1, " Force position 2", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Button);

    XPSetWidgetProperty(g_forcePositionButton, xpProperty_ButtonType, xpPushButton);
    XPHideWidget(g_forcePositionButton);
    XPSetWidgetProperty(g_forcePositionButton2, xpProperty_ButtonType, xpPushButton);
    XPHideWidget(g_forcePositionButton2);

    g_widgetListLogs = XPCreateCustomWidget( x+70, y-170-dy, x2-70, y-300-dy,
                                         1, "", 0, g_hlaWidgetsWidget, XPListBoxProc );

    g_labelOtherPlanesToDraw = XPCreateWidget(x+130, y-310-dy, x+270, y-332-dy,
                    1, " Other planes to draw : ", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Caption);

    XPHideWidget(g_labelOtherPlanesToDraw);

    g_widgetListOtherPlanes = XPCreateCustomWidget( x+70, y-340-dy, x2-70, y-440-dy,
                                                    1, "", 0, g_hlaWidgetsWidget, XPListBoxProc );

    XPHideWidget(g_widgetListOtherPlanes);

    g_labelOtherPlaneDescription = XPCreateWidget(x+130, y-450-dy, x+270, y-462-dy,
                    1, " Other plane description : ", 0, g_hlaWidgetsWidget,
                    xpWidgetClass_Caption);

    XPHideWidget(g_labelOtherPlaneDescription);

    g_widgetListOtherPlaneDescription = XPCreateCustomWidget( x+70, y-470-dy, x2-70, y-600-dy,
                                                              1, "", 0, g_hlaWidgetsWidget, XPListBoxProc );

    XPHideWidget(g_widgetListOtherPlaneDescription);


    XPAddWidgetCallback(g_hlaWidgetsWidget, HlaWidgetsHandler);
}

/**
 * @brief CreateMyPlaneFlightDynamicsWidgets
 *          Create Flight Dynamics information widget
 * @param x Coord X
 * @param y Coord Y
 * @param w Width
 * @param h Height
 */
void CreateMyPlaneFlightDynamicsWidgets(int x, int y, int w, int h)
{
    int x2 = x + w;
    int y2 = y - h;

    int dy = -50;

    g_myPlaneFlightDynamicsWidgetsWidget = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "My plane information",	// desc
                    1,		// root
                    NULL,	// no container
                    xpWidgetClass_MainWindow);

    XPSetWidgetProperty(g_myPlaneFlightDynamicsWidgetsWidget, xpProperty_MainWindowType, 1);
    XPSetWidgetProperty(g_myPlaneFlightDynamicsWidgetsWidget, xpProperty_MainWindowHasCloseBoxes, 1);

    g_myPlaneFlightDynamicsWidgetsWindow = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "",	// desc
                    0,		// root
                    g_myPlaneFlightDynamicsWidgetsWidget,
                    xpWidgetClass_SubWindow);

    XPSetWidgetProperty(g_myPlaneFlightDynamicsWidgetsWindow, xpProperty_SubWindowType, xpSubWindowStyle_SubWindow);


    g_labelMyPlaneFlightDynamicsModel = XPCreateWidget(x+20, y-110-dy, x+270, y-132-dy,
                    1, " Flight dynamics model : ", 0, g_myPlaneFlightDynamicsWidgetsWidget,
                    xpWidgetClass_Caption);
    g_widgetMyPlaneFlightDynamicsModel = XPCreateCustomWidget( x+20, y-140-dy, x2-20, y-540-dy,
                                                    1, "", 0, g_myPlaneFlightDynamicsWidgetsWidget, XPListBoxProc );


    g_labelMyPlaneTimeInformationModelData = XPCreateWidget(x+20, y-560-dy, x+270, y-572-dy,
                    1, " Time information : ", 0, g_myPlaneFlightDynamicsWidgetsWidget,
                    xpWidgetClass_Caption);
    g_widgetMyPlaneTimeInformationModelData = XPCreateCustomWidget( x+20, y-590-dy, x2-20, y-620-dy,
                                                              1, "", 0, g_myPlaneFlightDynamicsWidgetsWidget, XPListBoxProc );


    XPAddWidgetCallback(g_myPlaneFlightDynamicsWidgetsWidget, myPlaneFlightDynamicsWidgetsHandler);
}

/**
 * @brief CreateMyPlaneHydraulicActuatorsWidgets
 *          Create HydraulicActuartors inforrmation widget
 * @param x Coord X
 * @param y Coord Y
 * @param w Width
 * @param h Height
 */
void CreateMyPlaneHydraulicActuatorsWidgets(int x, int y, int w, int h)
{
    int x2 = x + w;
    int y2 = y - h;

    int dy = -50;

    g_myPlaneHydraulicActuatorsWidgetsWidget = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "My plane information",	// desc
                    1,		// root
                    NULL,	// no container
                    xpWidgetClass_MainWindow);

    XPSetWidgetProperty(g_myPlaneHydraulicActuatorsWidgetsWidget, xpProperty_MainWindowType, 1);
    XPSetWidgetProperty(g_myPlaneHydraulicActuatorsWidgetsWidget, xpProperty_MainWindowHasCloseBoxes, 1);

    g_myPlaneHydraulicActuatorsWidgetsWindow = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "",	// desc
                    0,		// root
                    g_myPlaneHydraulicActuatorsWidgetsWidget,
                    xpWidgetClass_SubWindow);

    XPSetWidgetProperty(g_myPlaneHydraulicActuatorsWidgetsWindow, xpProperty_SubWindowType, xpSubWindowStyle_SubWindow);


    g_labelMyPlaneHydraulicActuatorsModel = XPCreateWidget(x+20, y-110-dy, x+270, y-132-dy,
                    1, " Hydraulic actuators model : ", 0, g_myPlaneHydraulicActuatorsWidgetsWidget,
                    xpWidgetClass_Caption);
    g_widgetMyPlaneHydraulicActuatorsModel = XPCreateCustomWidget( x+20, y-140-dy, x2-20, y-440-dy,
                                                              1, "", 0, g_myPlaneHydraulicActuatorsWidgetsWidget, XPListBoxProc );

    XPAddWidgetCallback(g_myPlaneHydraulicActuatorsWidgetsWidget, myPlaneHydraulicActuatorsWidgetsHandler);
}

/**
 * @brief CreateMyPlaneEngineActuatorsWidgets
 *          Create Engine Actuators widget to display data
 * @param x Coord X
 * @param y Coord Y
 * @param w Width
 * @param h Height
 */
void CreateMyPlaneEngineActuatorsWidgets(int x, int y, int w, int h)
{
    int x2 = x + w;
    int y2 = y - h;

    int dy = -50;

    g_myPlaneEngineActuatorsWidgetsWidget = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "My plane information",	// desc
                    1,		// root
                    NULL,	// no container
                    xpWidgetClass_MainWindow);

    XPSetWidgetProperty(g_myPlaneEngineActuatorsWidgetsWidget, xpProperty_MainWindowType, 1);
    XPSetWidgetProperty(g_myPlaneEngineActuatorsWidgetsWidget, xpProperty_MainWindowHasCloseBoxes, 1);

    g_myPlaneEngineActuatorsWidgetsWindow = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "",	// desc
                    0,		// root
                    g_myPlaneEngineActuatorsWidgetsWidget,
                    xpWidgetClass_SubWindow);

    XPSetWidgetProperty(g_myPlaneEngineActuatorsWidgetsWindow, xpProperty_SubWindowType, xpSubWindowStyle_SubWindow);


    g_labelMyPlaneEngineActuatorsModel = XPCreateWidget(x+20, y-110-dy, x+270, y-132-dy,
                    1, " Engine actuators model : ", 0, g_myPlaneEngineActuatorsWidgetsWidget,
                    xpWidgetClass_Caption);
    g_widgetMyPlaneEngineActuatorsModel = XPCreateCustomWidget( x+20, y-140-dy, x2-20, y-840-dy,
                                                              1, "", 0, g_myPlaneEngineActuatorsWidgetsWidget, XPListBoxProc );

    XPAddWidgetCallback(g_myPlaneEngineActuatorsWidgetsWidget, myPlaneEngineActuatorsWidgetsHandler);
}

/**
 * @brief CreateVolFormationWidgets
 *          Create Flight Formation widget to force other plane to flight in formation.
 *          Current Plane will be the plane 0 in the formation
 * @param x Coord X
 * @param y Coord Y
 * @param w Width
 * @param h Height
 */
void CreateVolFormationWidgets(int x, int y, int w, int h)
{
    int x2 = x + w;
    int y2 = y - h;

    int dy = -50;

    g_myPlaneVolFormationWidgetsWidget = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "My plane information",	// desc
                    1,		// root
                    NULL,	// no container
                    xpWidgetClass_MainWindow);

    XPSetWidgetProperty(g_myPlaneVolFormationWidgetsWidget, xpProperty_MainWindowType, 1);
    XPSetWidgetProperty(g_myPlaneVolFormationWidgetsWidget, xpProperty_MainWindowHasCloseBoxes, 1);

    g_myPlaneVolFormationWidgetsWindow = XPCreateWidget(x, y, x2, y2,
                    1,	// Visible
                    "",	// desc
                    0,		// root
                    g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_SubWindow);

    XPSetWidgetProperty(g_myPlaneVolFormationWidgetsWindow, xpProperty_SubWindowType, xpSubWindowStyle_SubWindow);


    g_labelLongitudeModel = XPCreateWidget(x+60, y-110-dy, x+100, y-132-dy,
                    1, " Longitude : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldLongitudeModel = XPCreateWidget(x+140, y-110-dy, x2-70, y-132-dy,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldLongitudeModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelLatitudeModel = XPCreateWidget(x+60, y-110-dy-20, x+100, y-132-dy-20,
                    1, " Latitude : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldLatitudeModel = XPCreateWidget(x+140, y-110-dy-20, x2-70, y-132-dy-20,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldLatitudeModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelAltitudeModel = XPCreateWidget(x+60, y-110-dy-40, x+100, y-132-dy-40,
                    1, " Altitude : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldAltitudeModel = XPCreateWidget(x+140, y-110-dy-40, x2-70, y-132-dy-40,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldAltitudeModel, xpProperty_TextFieldType, xpTextEntryField);

//    g_labelVitesseModel = XPCreateWidget(x+60, y-110-dy-60, x+100, y-132-dy-60,
//                    1, " Vitesse : ", 0, g_myPlaneVolFormationWidgetsWidget,
//                    xpWidgetClass_Caption);
//    g_textFieldVitesseModel = XPCreateWidget(x+140, y-110-dy-60, x2-70, y-132-dy-60,
//                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
//                            xpWidgetClass_TextField);
//    XPSetWidgetProperty(g_textFieldVitesseModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelCapModel = XPCreateWidget(x+60, y-110-dy-60, x+100, y-132-dy-60,
                    1, " Cap : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldCapModel = XPCreateWidget(x+140, y-110-dy-60, x2-70, y-132-dy-60,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldCapModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelVelocityXModel = XPCreateWidget(x+60, y-110-dy-80, x+100, y-132-dy-80,
                    1, " Velocity X : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldVelocityXModel = XPCreateWidget(x+140, y-110-dy-80, x2-70, y-132-dy-80,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldVelocityXModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelVelocityYModel = XPCreateWidget(x+60, y-110-dy-100, x+100, y-132-dy-100,
                    1, " Velocity Y : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldVelocityYModel = XPCreateWidget(x+140, y-110-dy-100, x2-70, y-132-dy-100,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldVelocityYModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelVelocityZModel = XPCreateWidget(x+60, y-110-dy-120, x+100, y-132-dy-120,
                    1, " Velocity Z : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldVelocityZModel = XPCreateWidget(x+140, y-110-dy-120, x2-70, y-132-dy-120,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldVelocityZModel, xpProperty_TextFieldType, xpTextEntryField);

    g_labelSeparatorModel = XPCreateWidget(x+60, y-110-dy-140, x+100, y-132-dy-140,
                    1, "------------------------------------------------------------", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);

    g_labelDistanceModel = XPCreateWidget(x+60, y-110-dy-170, x+100, y-132-dy-170,
                    1, " Distance : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);
    g_textFieldDistanceModel = XPCreateWidget(x+140, y-110-dy-170, x2-70, y-132-dy-170,
                            1, "", 0, g_myPlaneVolFormationWidgetsWidget,
                            xpWidgetClass_TextField);
    XPSetWidgetProperty(g_textFieldDistanceModel, xpProperty_TextFieldType, xpTextEntryField);
    std::ostringstream distanceStr;
    distanceStr << 0.0001;
    XPSetWidgetDescriptor(g_textFieldDistanceModel, distanceStr.str().c_str());


    g_labelFormationModel = XPCreateWidget(x+60, y-110-dy-190, x+100, y-132-dy-190,
                    1, " Formation : ", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Caption);

    g_listFormationModel = XPCreateCustomWidget( x+140, y-115-dy-210, x2-50, y-175-dy-210,
                                                 1, "", 0, g_myPlaneVolFormationWidgetsWidget, XPListBoxProc );
    XPSetWidgetProperty(g_listFormationModel, xpProperty_TextFieldType, xpTextEntryField);
    addItemInListFormation("Colonne");
    addItemInListFormation("Ligne");
    addItemInListFormation("Echelon droit");
    addItemInListFormation("Echelon gauche");
    addItemInListFormation("Triangle");

    g_volFormationButtonInitialize = XPCreateWidget(x+30, y-180-dy-230, x+230, y-202-dy-230,
                    1, " Initialiser un vol en formation", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Button);
    XPSetWidgetProperty(g_volFormationButtonInitialize, xpProperty_ButtonType, xpPushButton);

    g_volFormationButtonInitialize2 = XPCreateWidget(x+250, y-180-dy-230, x+450, y-202-dy-230,
                    1, " Initialiser un vol en formation", 0, g_myPlaneVolFormationWidgetsWidget,
                    xpWidgetClass_Button);
    XPSetWidgetProperty(g_volFormationButtonInitialize2, xpProperty_ButtonType, xpPushButton);

    XPAddWidgetCallback(g_myPlaneVolFormationWidgetsWidget, volFormationWidgetsHandler);
}

/**
 * @brief Callback
 *          These callback is call by XplaneSdk at each frame
 * @return -1 to be called at the next cylcle
 */
static float Callback
(float /*sinceCall*/, float /*sinceLoop*/, int /*counter*/, void */*ref*/)
{
    if(federate)
    {
        federate->getfromXplaneLocalHLAFlightDynamicsModelData();
        federate->getfromXplaneLocalHLAEngineActuatorsModelData();
        federate->getfromXplaneLocalHLAHydraulicActuatorsModelData();
        federate->getfromXplaneLocalHLATimeInformationModelData();

        if(g_menuFlightDynamicsIsVisible)
        {
            updateFlightDynamicData();
            updateTimeInformationData();
        }
        if(g_menuHydraulicActuatorsIsVisible)
        {
            updateHydraulicActuatorData();
        }
        if(g_menuEngineActuatorsIsVisible)
        {
            updateEngineActuatorsData();
        }

        if(federate->isInitialized())
        {
            // Refresh the list which contain planes presence
            if(g_nbOtherPlaneToDraw != federate->getNbOtherAircraftToDraw())
            {
                clearListOtherPlanes();
                auto aircrafts = federate->getOthersAircraftToDraw();

                for(auto it = aircrafts.begin(); it != aircrafts.end(); ++it)
                {
                    addItemInListOtherPlanes(it->second.aircraft_name);
                }
                g_nbOtherPlaneToDraw = federate->getNbOtherAircraftToDraw();
                if(federate->isCreator())
                {
                    if(g_nbOtherPlaneToDraw == 0)
                    {
                        XPShowWidget(g_disconnectFederationButton);
                        g_planeSelected = "";
                        clearListOtherPlanes();
                    }
                    else
                    {
                        XPHideWidget(g_disconnectFederationButton);
                    }
                }
            }
            if(g_planeSelected != "")
            {
                updateAircraftsItems(g_planeSelected);
            }

            federate->sendUpdateAttributes();
            federate->runOneStep();
        }
    }

  return -1.0;
}

/**
 * @brief menu_handler
 *          Handle when user click on a menu
 * @param in_item_ref The id of the menu
 */
void menu_handler(void * /*in_menu_ref*/, void * in_item_ref)
{
    if(!strcmp((const char *)in_item_ref, "Open"))
    {
        if (g_menuIHLAIsCreated == 0)
        {
            CreateHlaWidgets(300, 650, 450, 650);
            g_menuIHLAIsCreated = 1;
        }
        else
            if(!XPIsWidgetVisible(g_hlaWidgetsWidget))
                XPShowWidget(g_hlaWidgetsWidget);
    }
    else if(!strcmp((const char *)in_item_ref, "Flight Dynamics"))
    {
        if (g_menuItemFlightDynamicsIsCreated == 0)
        {
            CreateMyPlaneFlightDynamicsWidgets(300, 650, 450, 650);
            g_menuItemFlightDynamicsIsCreated = 1;
            g_menuFlightDynamicsIsVisible = true;
        }
        else
            if(!XPIsWidgetVisible(g_myPlaneFlightDynamicsWidgetsWidget)) {
                XPShowWidget(g_myPlaneFlightDynamicsWidgetsWidget);
                g_menuFlightDynamicsIsVisible = true;
            }
    }
    else if(!strcmp((const char *)in_item_ref, "Hydraulic Actuators"))
    {
        if (g_menuItemHydraulicActuatorsIsCreated == 0)
        {
            CreateMyPlaneHydraulicActuatorsWidgets(300, 650, 350, 450);
            g_menuItemHydraulicActuatorsIsCreated = 1;
            g_menuHydraulicActuatorsIsVisible = true;
        }
        else
            if(!XPIsWidgetVisible(g_myPlaneHydraulicActuatorsWidgetsWidget)) {
                XPShowWidget(g_myPlaneHydraulicActuatorsWidgetsWidget);
                g_menuHydraulicActuatorsIsVisible = true;
            }
    }
    else if(!strcmp((const char *)in_item_ref, "Engine Actuators"))
    {
        if (g_menuItemEngineActuatorsIsCreated == 0)
        {
            CreateMyPlaneEngineActuatorsWidgets(300, 650, 350, 950);
            g_menuItemEngineActuatorsIsCreated = 1;
            g_menuEngineActuatorsIsVisible = true;
        }
        else
            if(!XPIsWidgetVisible(g_myPlaneEngineActuatorsWidgetsWidget)) {
                XPShowWidget(g_myPlaneEngineActuatorsWidgetsWidget);
                g_menuEngineActuatorsIsVisible = true;
            }
    }
    else if(!strcmp((const char *)in_item_ref, "Vol Formation"))
    {
        if (g_menuVolFormationIsCreated == 0)
        {
            CreateVolFormationWidgets(300, 650, 500, 450);
            g_menuVolFormationIsCreated = 1;
            g_menuVolFormationIsVisible = true;
        }
        else
            if(!XPIsWidgetVisible(g_myPlaneVolFormationWidgetsWidget)) {
                XPShowWidget(g_myPlaneVolFormationWidgetsWidget);
                g_menuVolFormationIsVisible = true;
            }
    }
}

PLUGIN_API int XPluginStart (char *name, char *signature, char *description)
{
	strcpy (name, "ISAE-SUPAERO HLA/CERTI Plugin");
	strcpy (signature, "ISAE-SUPAERO");
	strcpy (description, "HLA plugin for co-simulation with HLA/CERTI.");

	int xplane_ver;
	int sdk_ver;
	XPLMHostApplicationID app_id;
	XPLMGetVersions (&xplane_ver, &sdk_ver, &app_id);

    // Used by widget to make sure only one widgets instance created
    g_menuIHLAIsCreated = 0;
    g_menuItemFlightDynamicsIsCreated = 0;
    g_menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "HLA", 0, 0);
    g_menu_id = XPLMCreateMenu("HLA", XPLMFindPluginsMenu(), g_menu_container_idx, menu_handler, NULL);
    XPLMAppendMenuItem(g_menu_id, "Open", (void *)"Open", 1);
    XPLMAppendMenuItem(g_menu_id, "Flight Dynamics", (void *)"Flight Dynamics", 1);
    XPLMAppendMenuItem(g_menu_id, "Hydraulic Actuators", (void *)"Hydraulic Actuators", 1);
    XPLMAppendMenuItem(g_menu_id, "Engine Actuators", (void *)"Engine Actuators", 1);
    XPLMAppendMenuItem(g_menu_id, "Vol Formation", (void *)"Vol Formation", 1);

    federate = std::make_unique<XplaneFederateHla1516e>();

    XPLMRegisterFlightLoopCallback (Callback, -1.0, NULL);

	return 1;
}

PLUGIN_API void XPluginStop (void)
{
    XPLMDestroyMenu(g_menu_id);

    if (g_menuIHLAIsCreated == 1)
    {
        XPDestroyWidget(g_hlaWidgetsWidget, 1);
        g_menuIHLAIsCreated = 0;
    }

    if (g_menuItemFlightDynamicsIsCreated == 1)
    {
        XPDestroyWidget(g_myPlaneFlightDynamicsWidgetsWidget, 1);
        g_menuItemFlightDynamicsIsCreated = 0;
    }
}

PLUGIN_API void XPluginDisable (void)
{

}

PLUGIN_API int XPluginEnable (void)
{

    return 1;
}

PLUGIN_API void XPluginReceiveMessage (XPLMPluginID from, long message, void *param)
{
	if (from == XPLM_PLUGIN_XPLANE) {
	switch (message) 
	{
		case XPLM_MSG_PLANE_LOADED:
		  if (param == XPLM_PLUGIN_XPLANE) {
		  }
		  break;
		default:
		  break;
		}	
	}
}
