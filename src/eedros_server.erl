%%%-------------------------------------------------------------------
%%% @author Will Vining <wfvining@cs.unm.edu>
%%% @copyright (C) 2018, Will Vining
%%% @doc
%%%
%%% @end
%%% Created : 30 May 2018 by Will Vining <wfvining@cs.unm.edu>
%%%-------------------------------------------------------------------
-module(eedros_server).

-behaviour(gen_server).

%% API
-export([start_link/1,
         advertise/1,
         subscribe/1,
         unsubscribe/1,
         publish/2]).

%% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
         terminate/2, code_change/3, format_status/2]).

-define(SERVER, ?MODULE).

-record(state, {subscriptions, sup}).

%%%===================================================================
%%% API
%%%===================================================================

%%--------------------------------------------------------------------
%% @doc
%% Starts the server
%% @end
%%--------------------------------------------------------------------
-spec start_link(Supervisor :: pid()) -> {ok, Pid :: pid()} |
                      {error, Error :: {already_started, pid()}} |
                      {error, Error :: term()} |
                      ignore.
start_link(Supervisor) ->
    gen_server:start_link({local, ?SERVER}, ?MODULE, Supervisor, []).


%%--------------------------------------------------------------------
%% @doc
%% Unsubscribe from a topic
%% @end
%%--------------------------------------------------------------------
-spec advertise(Topic :: string()) -> ok.
advertise(Topic) ->
    gen_server:call(?SERVER, {advertise, Topic}).

%%--------------------------------------------------------------------
%% @doc
%% Subscribe to a topic
%% @end
%%--------------------------------------------------------------------
-spec subscribe(Topic :: string()) -> ok.
subscribe(Topic) ->
    gen_server:call(?SERVER, {subscribe, Topic}).

%%--------------------------------------------------------------------
%% @doc
%% Unsubscribe from a topic
%% @end
%%--------------------------------------------------------------------
-spec unsubscribe(Topic :: string()) -> ok.
unsubscribe(Topic) ->
    gen_server:cast(?SERVER, {unsubscribe, Topic}).

%%--------------------------------------------------------------------
%% @doc
%% Publish a message on a topic
%%
%% For now the message will be a string that can be directly
%% interpreted by 'rostopic', but in the future is should be a binary.
%%
%% @end
%% --------------------------------------------------------------------
-spec publish(Topic :: string(), Message :: binary()) -> ok.
publish(Topic, Message) ->
    gen_server:cast(?SERVER, {publish, Topic, Message}).


%%%===================================================================
%%% gen_server callbacks
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Initializes the server
%% @end
%%--------------------------------------------------------------------
-spec init(Args :: term()) -> {ok, State :: term()} |
                              {ok, State :: term(), Timeout :: timeout()} |
                              {ok, State :: term(), hibernate} |
                              {stop, Reason :: term()} |
                              ignore.
init(Supervisor) ->
    process_flag(trap_exit, true),
    self() ! {start_advertisement_supervisor, Supervisor},
    {ok, #state{}}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling call messages
%% @end
%%--------------------------------------------------------------------
-spec handle_call(Request :: term(), From :: {pid(), term()}, State :: term()) ->
                         {reply, Reply :: term(), NewState :: term()} |
                         {reply, Reply :: term(), NewState :: term(), Timeout :: timeout()} |
                         {reply, Reply :: term(), NewState :: term(), hibernate} |
                         {noreply, NewState :: term()} |
                         {noreply, NewState :: term(), Timeout :: timeout()} |
                         {noreply, NewState :: term(), hibernate} |
                         {stop, Reason :: term(), Reply :: term(), NewState :: term()} |
                         {stop, Reason :: term(), NewState :: term()}.
handle_call({subscribe, Topic, Type}, From, State) ->
    %Handler = ebus:sub(),
    Reply = ok,
    {reply, Reply, State};
handle_call({advertise, Topic}, _From, State) ->
    Reply = advertisement_sup:advertise(Topic),
    {reply, Reply, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling cast messages
%% @end
%%--------------------------------------------------------------------
-spec handle_cast(Request :: term(), State :: term()) ->
                         {noreply, NewState :: term()} |
                         {noreply, NewState :: term(), Timeout :: timeout()} |
                         {noreply, NewState :: term(), hibernate} |
                         {stop, Reason :: term(), NewState :: term()}.
handle_cast({publish, Topic, Message}, State) ->
    ebus:pub(Topic, Message),
    {noreply, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling all non call/cast messages
%% @end
%%--------------------------------------------------------------------
-spec handle_info(Info :: timeout() | term(), State :: term()) ->
                         {noreply, NewState :: term()} |
                         {noreply, NewState :: term(), Timeout :: timeout()} |
                         {noreply, NewState :: term(), hibernate} |
                         {stop, Reason :: normal | term(), NewState :: term()}.
handle_info({start_advertisement_supervisor, Supervisor}, State) ->

    AdvSpec = #{ id       => advertisement_sup,
                 start    => {advertisement_sup, start_link, []},
                 restart  => permanent,
                 shutdown => 10000,
                 type     => supervisor },

    {ok, AdvSup} = supervisor:start_child(Supervisor, AdvSpec),
    { noreply, State#state{ sup = AdvSup } }.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% This function is called by a gen_server when it is about to
%% terminate. It should be the opposite of Module:init/1 and do any
%% necessary cleaning up. When it returns, the gen_server terminates
%% with Reason. The return value is ignored.
%% @end
%%--------------------------------------------------------------------
-spec terminate(Reason :: normal | shutdown | {shutdown, term()} | term(),
                State :: term()) -> any().
terminate(_Reason, _State) ->
    ok.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Convert process state when code is changed
%% @end
%%--------------------------------------------------------------------
-spec code_change(OldVsn :: term() | {down, term()},
                  State :: term(),
                  Extra :: term()) -> {ok, NewState :: term()} |
                                      {error, Reason :: term()}.
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% This function is called for changing the form and appearance
%% of gen_server status when it is returned from sys:get_status/1,2
%% or when it appears in termination error logs.
%% @end
%%--------------------------------------------------------------------
-spec format_status(Opt :: normal | terminate,
                    Status :: list()) -> Status :: term().
format_status(_Opt, Status) ->
    Status.

%%%===================================================================
%%% Internal functions
%%%===================================================================
