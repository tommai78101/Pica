#include "physics.h"

/*
 * LISTENER.C
 */

/**
 * @brief This is where you write your very own Listener_Init() function. This function's purpose is to initialize your C3D_ContactListener object.
 * @note By default, the default virtual method table (v-table) has been given. If you wish to use your own methods and implementations, you need to use the struct object,
 *       C3D_ContactListener_FuncTable, to store your function pointers, then pass it to the C3D_ContactListener object's v-table.
 * @param[in,out]        this            Expect to pass in a pointer to the C3D_ContactListener object, and fill in or initialize the data structure. 
 */
void Listener_Init(C3D_ContactListener* listener)
{
	listener->vmt = &Listener_Default_VMT;
}

/**
 * @brief This is where you write your very own Listener_Free() function. This function's purpose is to deallocate/release/free up resources from your C3D_ContactListener object.
 * @note By default, this function is not implemented. Either you are required to virtualize this function, or just let it do nothing.
 * @param[in,out]        this            Expect to pass in a pointer to the C3D_ContactListener object, and find a way to free up the resources, or check to see if there are no 
 *                                       existing resources left.
 */
void Listener_Free(C3D_ContactListener* listener)
{
	//Listener_Free() - Unimplemented method: Requires virtualizing this function, or just leave it empty. If you do any memory allocation in the Listener_Init(), you must explicitly
	//                  handle memory deallocation on your own, and that must be implemented here.
}

/**
 * @brief This is where you write your very own Listener_BeginContact() function. This function's purpose is to initiate handling events of which a C3D_ContactConstraint object is to be 
 *        listened, or when an event has been received.
 * @param[in,out]        this            Expect a pointer to a C3D_ContactListener object where you update your data that depends on this event.
 * @param[in]            constraint      A C3D_ContactConstraint object that, when the event has been listened to, gives you a snapshot of the contact constraint you are listening to.
 */
void Listener_BeginContact(C3D_ContactListener* listener, const C3D_ContactConstraint* constraint)
{
	listener->vmt->BeginContact(listener, constraint);
}

/**
 * @brief This is where you write your very own Listener_EndContact() function. This function's purpose is to finish handling events of which a C3D_ContactConstraint object is no longer to be
 *        listened, or when an event has finished its course of action.
 * @param[in,out]        this            Expect a pointer to a C3D_ContactListener object where you update your data that depends on this event.
 * @param[in]            constraint      A C3D_ContactConstraint object that, when the event has finished, gives you a snapshot of the contact constraint you were listening to. 
 */
void Listener_EndContact(C3D_ContactListener* listener, const C3D_ContactConstraint* constraint)
{
	listener->vmt->EndContact(listener, constraint);
}
