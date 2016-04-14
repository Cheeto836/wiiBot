/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> //definitions for data types used in system calls
#include <sys/socket.h> //definitions of structs needed for sockets
#include <netinet/in.h> //constants and structs needed for ID addresses
#include "wiicpp.h"

void error(const char *msg)
{
  //function is called when a system call fails.
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[])
{
  //create file descriptors (array subscripts into the file descriptor table)
  int sockfd; //returned by socket system call
  int newsockfd; //returned by accept system call

  int portno; //port number on which the server accepts the connections
  socklen_t clilen; //size of the address of the client (needed for accept)
  int n; //returned value for read() and write() (num of chars read/written)
  char buffer[256]; //server reads chars from the socket connection to here

  /* sockaddr_in is a structure that contains an internet address.
   * the structure is defined in netinet/in.h
   * in_addr (defined in netinet/in.h) contains only one field, the field
   * is an unsigned long called s_addr
   */
  struct sockaddr_in serv_addr; //address of server
  struct sockaddr_in cli_addr; //address of client that connects to server
  
  if (argc < 2)
  {
    fprintf(stderr, "ERROR, no port provided\n");
    exit(1);
  }
  
  /* socket() creates a new socket. The function takes three arguments.
   *
   * First argument: the address domain of the socket. 
   * Two types of address domains: unix and internet
   *    unix domain: Used for two processes which share a common file system
   *                 Use the constant AF_UNIX as the argument
   *    internet domain: used for any two hosts on the internet
   *                     use the constant AF_INET as the argument
   *
   * Second argument: type of socket.
   * Two types of sockets: stream and datagram
   *    stream: characters read in a continuous stream as if from a file or
   *            pipeline.
   *            Use SOCK_STREAM as the argument.
   *    datagram: messages are read in chunks.
   *              Use SOCK_DGRAM as the argument.
   *
   * Third Argument: protocol.
   * If the argument is a zero (which is the usual case) the OS will choose
   * the most appropriate protocol. It will choose TCP for stream and UDP for
   * datagram sockets.
   *
   * The function returns and entry into the file descriptor table (small
   * integer). Thatvalue is used for all subsequent references to the socket.
   * If the socket call faile, the value returned will be -1.
   *
   * This is a simplified description of the socket call. For more info
   * reffer to the socket() man page.
   */
  sockfd =socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd <0)
    error("ERROR opening socket");

  /* bzero() sets all values in a buffer to zero. It takes two arguments.
   * First argument: pointer to the buffer.
   * Second argument: size of the buffer.
   */
  bzero((char*) &serv_addr, sizeof(serv_addr)); //inits serv_addr to zeros
  portno = atoi(argv[1]); //port number on which the server will listen

  /* serv_addr is a structure of type "struct sockaddr_in". This structure
   * contains four fields.
   * 
   * First field: short sin_family
   *    contains a code for the address family
   * Second field: unsigned short sin_port
   *    contains the port number. The value cannot simply be copied though,
   *    it is necessary to convert the port number from host byte order
   *    to network byte ofer. This is done with a call to htons()
   * Third field: struct in_addr
   *    contains the single field: unsigned long s_addr.
   *      This field contains the IP address of the host. For server code,
   *      this value will always be the IP address of the machine the code
   *      is running on. To get this value use the symbolic constant
   *      INADDR_ANY.
   */
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(portno);
  serv_addr.sin_addr.s_addr = INADDR_ANY;

  /* bind() binds a socket to an address. It takes three arguments.
   * First argument: socket's file descriptor.
   * Second argument: address on which to bind
   *    The argument is of type "struct sockaddr" but the actual input
   *    in this code is of type "struct sockaddr_in" and therefor must be
   *    correctly cast.
   * Third argument: size of the address on which to bind.
   *
   * This function can fail for a number of reasons, the most obvious being
   * that the socket is already in use.
   */
  if (bind(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0)
    error ("ERROR on binding");

  /* listen() allows a process to listen on the socket for connections.
   * First argument: socket file descriptor
   * Second argument: size ofthe backlog queue
   *    This represents the number of connections that can be waiting while
   *    the process is handling a particular connection. This value should
   *    be set to 5, the maximum size permitted by most systems. 
   * As long as the first argument is a valid socket, the code cannot fail
   * and there is no code to check for errors
   */
  listen(sockfd, 5);

  /* accept() causes the process to block until a client connects to the
   * server. It wakes up the process when a connection from a client has
   * been successfully established. The function returns a new file
   * descriptor. All communication done on this connection should use this
   * new file descriptor.
   * Second argument: referencepointer to the address of the client on the
   *                  other end of the connection.
   * Third argument: size of the structure
   */
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd, (struct sockaddr*) &cli_addr, &clilen);
  if (newsockfd < 0)
    error("ERROR on accept");

  /* This code can only be reached after a client has successfully connected
   * to the server. 
   * First step: initilize the read buffer using bzero().
   * Second step: read from the client using the read() function.
   *    The read() function call uses the new file descriptor (returned by
   *    accept).
   *    read() will block until something is there to be read (i.e. the
   *    client has called write()).
   *    Read returns the number of characters read.
   *    If the ammount of data sent is greater than the specified buffer size
   *    read() will only read the first 255 characters.
   */
  bzero(buffer, 256);
  for (int i = 0; i < 2; i++)
  {
    n = read(newsockfd, buffer, 10);
    if (n < 0)
      error("ERROR reading from socket");
    printf("Here is the message: %s\n", buffer);
  }

  n = write(newsockfd, "I got your message", 18);
  if (n < 0)
    error("ERROR writing to socket");

  return 0;
}
